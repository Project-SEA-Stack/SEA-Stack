/*********************************************************************
 * @file  setup_from_yaml.cpp
 * @brief Creates HydroSystem from parsed YAML configuration.
 *
 * Implements SetupHydroFromYAML(): matches YAML body configs to Chrono
 * bodies, creates wave models, and returns a configured HydroSystem.
 *********************************************************************/

#include <seastack/adapters/chrono/setup_from_yaml.h>
#include <seastack/hydro/config/config_loader.h>
#include <seastack/adapters/chrono/hydro_system.h>
#include <chrono/physics/ChSystem.h>
#ifdef SEASTACK_HAVE_MOORDYN
#include <seastack/mooring/moordyn_config.h>
#endif
#include <seastack/hydro/waves/wave_base.h>
#include <seastack/hydro/waves/linear_directional_wave_field.h>
#include <seastack/hydro/waves/eta_table_wave_field.h>
#include <seastack/hydro/waves/component_sampler.h>
#include <seastack/hydro/waves/wave_component.h>
#ifdef SEASTACK_HAVE_HYDRO_IO
#include <seastack/hydro_io/h5_reader.h>
#endif
#include <seastack/core/math_constants.h>
#include <seastack/infra/logging.h>
#include <seastack/hydro/radiation_types.h>
#include <algorithm>
#include <cctype>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace seastack::chrono {

using namespace ::chrono;
using namespace seastack::hydro;
#ifdef SEASTACK_HAVE_MOORDYN
using seastack::mooring::MoorDynConfig;
#endif

namespace {

constexpr int kDefaultSeed = 42;
constexpr double kDefaultGravity = 9.81;

std::string NormalizeHydroConfigToken(std::string s) {
    s.erase(std::remove_if(s.begin(), s.end(),
                           [](unsigned char c) { return std::isspace(c) != 0; }),
            s.end());
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return s;
}

/// Validates hydrodynamics YAML for imported eta (waves.eta_file): waves.method is
/// only allowed with eta_file; unknown or misplaced tokens (e.g. frequency_domain
/// under waves) are rejected. For table-based import (waves.method irf_convolution),
/// rejects excitation.method frequency_domain (no discrete components) and rejects
/// duplicate irf_convolution on excitation when waves already select IRF table mode.
void ValidateWaveExcitationForEtaImport(const YAMLHydroData& data) {
    const auto& w = data.waves;
    if (w.eta_file.empty()) {
        if (!w.method.empty()) {
            throw std::runtime_error(
                "hydrodynamics.waves.method is only valid when waves.eta_file is set");
        }
        return;
    }

    const std::string wm = NormalizeHydroConfigToken(w.method);
    if (!wm.empty() && wm != "legacy" && wm != "dft" && wm != "dft_components" &&
        wm != "irf_convolution" && wm != "irf" && wm != "frequency_domain") {
        throw std::runtime_error(
            "hydrodynamics.waves.method: unknown value '" + w.method +
            "' for eta_file import (expected irf_convolution, legacy/dft, or omit)");
    }
    if (wm == "frequency_domain") {
        throw std::runtime_error(
            "hydrodynamics.waves.method: frequency_domain is not valid under waves; "
            "omit waves.method for DFT eta import and use hydrodynamics.excitation if needed");
    }

    const std::string em = NormalizeHydroConfigToken(data.excitation_method);
    const bool eta_irf_table = (wm == "irf_convolution" || wm == "irf");
    if (eta_irf_table) {
        if (em == "frequency_domain" || em == "fd") {
            throw std::runtime_error(
                "hydrodynamics.excitation.method: frequency_domain is incompatible with "
                "waves.method: irf_convolution and eta_file (no discrete wave components)");
        }
        if (em == "irf_convolution" || em == "irf") {
            throw std::runtime_error(
                "hydrodynamics.excitation.method must not be set to irf_convolution when "
                "waves.method is irf_convolution; omit excitation.method or use auto");
        }
    }
}

// ------------------------------------------------------------
// SECTION: Wave model factory
// ------------------------------------------------------------

/// Build a SeaStateDefinition from the YAML WaveSettings.
SeaStateDefinition BuildSeaStateDefinition(const WaveSettings& ws) {
    SeaStateDefinition def;
    def.type  = ws.type;
    def.seed  = (ws.seed > 0) ? ws.seed : kDefaultSeed;
    def.depth = ws.depth;
    def.g     = kDefaultGravity;

    std::string type_lower = ws.type;
    std::transform(type_lower.begin(), type_lower.end(), type_lower.begin(), ::tolower);

    if (type_lower == "regular") {
        def.amplitude     = ws.height / 2.0;
        def.omega         = (ws.period > 0.0) ? (2.0 * M_PI / ws.period) : 0.0;
        def.direction_deg = ws.direction;
        def.phase_rad     = ws.phase;
        return def;
    }

    // Irregular: build partition(s).
    if (!ws.partitions.empty()) {
        // Explicit multi-partition (bimodal) config.
        for (const auto& p : ws.partitions) {
            SeaStatePartition sp;
            sp.spectrum.type = p.spectrum_type;
            sp.spectrum.Hs   = p.Hs;
            sp.spectrum.Tp   = p.Tp;
            sp.spectrum.gamma = p.gamma;
            sp.spreading.type = p.spreading.type;
            sp.spreading.mean_direction_deg = p.mean_direction_deg;
            sp.spreading.s = p.spreading.s;
            def.partitions.push_back(sp);
        }
    } else {
        // Single-partition shorthand (the common case).
        SeaStatePartition sp;
        sp.spectrum.Hs = ws.height;
        sp.spectrum.Tp = ws.period;
        sp.spectrum.gamma = ws.gamma;

        std::string spec_type = ws.spectrum;
        std::transform(spec_type.begin(), spec_type.end(), spec_type.begin(), ::tolower);
        sp.spectrum.type = spec_type;

        sp.spreading.type = ws.spreading.type;
        sp.spreading.mean_direction_deg = ws.direction;
        sp.spreading.s = ws.spreading.s;

        def.partitions.push_back(sp);
    }

    // Discretization.
    //
    // An explicit YAML setting always wins. Otherwise the default depends on
    // the wave source (see hydro_config.h): spectral seas use
    // kDefaultWaveSpectralNOmega, while eta-file imports must fall through to
    // ComponentSampler::Build's eta default (1000 Fourier bins). Leaving
    // def.n_omega = 0 here for eta imports preserves that larger DFT resolution;
    // forcing the spectral default would coarsen the reconstructed sea and
    // break eta-import regression baselines.
    if (ws.discretization.n_omega > 0) {
        def.n_omega = ws.discretization.n_omega;
    } else if (ws.nfrequencies > 0) {
        def.n_omega = ws.nfrequencies;
    } else if (!ws.eta_file.empty()) {
        def.n_omega = 0;  // ComponentSampler::Build applies the eta-import default
    } else {
        def.n_omega = kDefaultWaveSpectralNOmega;
    }
    def.n_theta = (ws.discretization.n_theta > 0) ? ws.discretization.n_theta : 1;

    // Frequency limits (convert Hz to rad/s if specified).
    if (ws.frequency_min > 0.0) def.omega_min = 2.0 * M_PI * ws.frequency_min;
    if (ws.frequency_max > 0.0) def.omega_max = 2.0 * M_PI * ws.frequency_max;

    return def;
}

/**
 * @brief Create a wave object from wave settings.
 *
 * Regular / irregular / bimodal seas use SeaStateDefinition + ComponentSampler +
 * LinearDirectionalWaveField. Eta-file import uses either DFT components (legacy) or
 * EtaTableWaveField when waves.method is irf_convolution.
 */
std::shared_ptr<WaveBase> CreateWaveFromSettings(const WaveSettings& wave_settings,
                                                 double timestep,
                                                 double sim_duration,
                                                 double ramp_duration,
                                                 double h5_water_depth) {
    std::string type = wave_settings.type;
    std::transform(type.begin(), type.end(), type.begin(), ::tolower);

    if (type == "no_wave" || type == "still_ci" || type == "still") {
        seastack::infra::debug::LogDebug("Attached wave model: NoWave (still water)");
        return std::make_shared<NoWave>();
    }

    if (!wave_settings.eta_file.empty()) {
        const std::string wm = NormalizeHydroConfigToken(wave_settings.method);
        const bool use_eta_table =
            (wm == "irf_convolution" || wm == "irf");
        if (!wm.empty() && !use_eta_table && wm != "legacy" && wm != "dft" &&
            wm != "dft_components") {
            throw std::runtime_error(
                "CreateWaveFromSettings: waves.method '" + wave_settings.method +
                "' is not valid for eta_file (use irf_convolution or omit for legacy DFT)");
        }
        if (use_eta_table) {
            const double depth = (wave_settings.depth > 0.0) ? wave_settings.depth
                                                             : h5_water_depth;
            auto eta_waves =
                std::make_shared<EtaTableWaveField>(wave_settings.eta_file, depth);
            const double effective_ramp = (wave_settings.ramp_duration > 0.0)
                                              ? wave_settings.ramp_duration
                                              : ramp_duration;
            eta_waves->SetRampDuration(effective_ramp);
            seastack::infra::debug::LogDebug(
                std::string("Attached wave model: EtaTableWaveField, eta_file=") +
                wave_settings.eta_file);
            return eta_waves;
        }
    }

    // Build a unified SeaStateDefinition from YAML settings.
    SeaStateDefinition def = BuildSeaStateDefinition(wave_settings);

    // Eta-file import (legacy): DFT extraction in ComponentSampler::Build.
    if (!wave_settings.eta_file.empty()) {
        def.eta_file_path = wave_settings.eta_file;
    }

    auto components = ComponentSampler::Build(def);
    auto wave_field = std::make_shared<LinearDirectionalWaveField>(
        std::move(components), def.depth);

    double effective_ramp = (wave_settings.ramp_duration > 0.0)
                            ? wave_settings.ramp_duration
                            : ramp_duration;
    wave_field->SetRampDuration(effective_ramp);

    int n_comp = static_cast<int>(wave_field->GetComponents().size());
    seastack::infra::debug::LogDebug(
        std::string("Attached wave model: LinearDirectionalWaveField, ") +
        std::to_string(n_comp) + " components" +
        (wave_settings.eta_file.empty() ? "" : ", eta_file=" + wave_settings.eta_file));

    return wave_field;
}

/**
 * @brief Match hydrodynamic bodies with Chrono bodies by name.
 */
std::vector<std::shared_ptr<ChBody>> MatchBodiesByName(
    const std::vector<HydroBody>& hydro_bodies,
    const std::vector<std::shared_ptr<ChBody>>& chrono_bodies,
    std::string& h5_file_path) {
    
    std::vector<std::shared_ptr<ChBody>> matched_bodies;
    
    // For now, we'll use the first H5 file found (assuming all bodies use the same file)
    // In the future, this could be enhanced to support different H5 files per body
    if (!hydro_bodies.empty()) {
        h5_file_path = hydro_bodies[0].h5_file;
    }
    
    // Match bodies by name
    for (const auto& hydro_body : hydro_bodies) {
        bool found = false;
        
        for (const auto& chrono_body : chrono_bodies) {
            if (chrono_body->GetName() == hydro_body.name) {
                matched_bodies.push_back(chrono_body);
                found = true;
                
                // Log the matched body details
                seastack::infra::debug::LogDebug(std::string("Body: ") + hydro_body.name + 
                          " -> h5: " + h5_file_path + 
                          ", excitation: " + (hydro_body.include_excitation ? "true" : "false") + 
                          ", radiation: " + (hydro_body.include_radiation ? "true" : "false"));
                
                break;
            }
        }
        
        if (!found) {
            seastack::infra::cli::LogWarning("Hydrodynamic body '" + hydro_body.name + "' not found in Chrono system");
        }
    }
    
    return matched_bodies;
}

// ------------------------------------------------------------
// SECTION: Body matching
// ------------------------------------------------------------
// Matches YAML body configurations to Chrono bodies by name.

} // anonymous namespace

// ------------------------------------------------------------
// SECTION: Main setup function
// ------------------------------------------------------------
// Orchestrates body matching, wave creation, and HydroSystem initialization.

std::unique_ptr<HydroSystem> SetupHydroFromYAML(
    const YAMLHydroData& hydro_data,
    const std::vector<std::shared_ptr<ChBody>>& bodies,
    double timestep,
    double sim_duration,
    double ramp_duration) {
    
    // Match hydrodynamic bodies with Chrono bodies (multibody: matches all configured bodies)
    std::string h5_file_path;
    auto matched_bodies = MatchBodiesByName(hydro_data.bodies, bodies, h5_file_path);
    
    if (matched_bodies.empty()) {
        throw std::runtime_error("No hydrodynamic bodies found in Chrono system");
    }

    // Chrono's GMRES solver is sensitive to body ordering in the system body list
    // when ChLoadHydrodynamics (infinite-frequency added mass) is active. The
    // added-mass cross-coupling blocks must align with the system's velocity-DOF
    // offsets, which are assigned in body-list order. Ensure the matched hydro
    // bodies appear first in the system body list, in the same order as the H5
    // data, so offsets match the block layout expected by ChLoadHydrodynamics.
    if (auto* sys = matched_bodies[0]->GetSystem()) {
        std::vector<std::shared_ptr<ChBody>> non_hydro;
        for (const auto& b : sys->GetBodies()) {
            bool is_hydro = false;
            for (const auto& hb : matched_bodies)
                if (hb == b) { is_hydro = true; break; }
            if (!is_hydro)
                non_hydro.push_back(b);
        }

        // Only rearrange if current order doesn't already match.
        bool needs_reorder = false;
        const auto& current = sys->GetBodies();
        for (size_t i = 0; i < matched_bodies.size(); ++i) {
            if (i >= current.size() || current[i] != matched_bodies[i]) {
                needs_reorder = true;
                break;
            }
        }

        if (needs_reorder) {
            for (const auto& b : non_hydro) sys->RemoveBody(b);
            for (const auto& b : matched_bodies) sys->RemoveBody(b);
            for (const auto& b : matched_bodies) sys->AddBody(b);
            for (const auto& b : non_hydro) sys->AddBody(b);
            seastack::infra::debug::LogDebug(
                "Reordered system body list so hydro bodies come first (H5 order).");
        }
    }

    ValidateWaveExcitationForEtaImport(hydro_data);

    double h5_water_depth = 0.0;
#ifdef SEASTACK_HAVE_HYDRO_IO
    if (!hydro_data.waves.eta_file.empty()) {
        seastack::hydro_io::H5FileInfo h5peek(
            h5_file_path, static_cast<int>(matched_bodies.size()));
        seastack::hydro::HydroData hdpeek = h5peek.ReadH5Data();
        h5_water_depth = hdpeek.GetSimulationInfo().water_depth;
    }
#endif

    // Create wave object from settings (system-wide, not per-body)
    auto wave = CreateWaveFromSettings(hydro_data.waves, timestep, sim_duration,
                                       ramp_duration, h5_water_depth);
    
    // Create and initialize HydroSystem (multibody: all matched bodies passed in)
    const HydroCouplingOptions coupling;
    auto hydro_system =
        std::make_unique<HydroSystem>(matched_bodies, h5_file_path, wave, coupling);
    
    seastack::infra::debug::LogDebug(std::string("Initialized HydroSystem with ") + std::to_string(matched_bodies.size()) + " bodies");

    // ─────────────────────────────────────────────────────────────────────────
    // Per-body hydrostatics configuration
    // ─────────────────────────────────────────────────────────────────────────
    {
        using BodyHSConfig = seastack::hydro::HydroModelBuilder::BodyHydrostaticsConfig;
        std::vector<BodyHSConfig> hs_configs;
        hs_configs.reserve(hydro_data.bodies.size());
        int nl_count = 0;
        for (const auto& body : hydro_data.bodies) {
            BodyHSConfig cfg;
            cfg.model = body.hydrostatics.model;
            cfg.mesh_file = body.hydrostatics.mesh_file;
            hs_configs.push_back(std::move(cfg));
            if (body.hydrostatics.model == seastack::hydro::HydrostaticsModel::kNonlinear) {
                ++nl_count;
            }
        }
        hydro_system->SetBodyHydrostatics(std::move(hs_configs));

        // Log per-body hydrostatics mode
        for (const auto& body : hydro_data.bodies) {
            std::string mode_str = (body.hydrostatics.model == seastack::hydro::HydrostaticsModel::kNonlinear)
                ? "nonlinear" : "linear";
            seastack::infra::debug::LogDebug(
                std::string("Body '") + body.name + "' hydrostatics: " + mode_str);
        }
        if (nl_count > 0) {
            seastack::infra::cli::LogDebug(seastack::infra::cli::CreateAlignedLine(
                "+", "Nonlinear Hydrostatics",
                std::to_string(nl_count) + " of " +
                std::to_string(hydro_data.bodies.size()) + " bodies"));
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Excitation method (optional YAML override)
    //
    // Default kAuto is resolved in HydroModelBuilder: frequency-domain for
    // irregular seas with multiple distinct wave headings; IRF convolution for
    // long-crested irregular only.  YAML may force irf_convolution or
    // frequency_domain when needed.
    // ─────────────────────────────────────────────────────────────────────────
    {
        std::string m = hydro_data.excitation_method;
        m.erase(std::remove_if(m.begin(), m.end(),
                               [](unsigned char c) { return std::isspace(c) != 0; }),
                m.end());
        std::transform(m.begin(), m.end(), m.begin(), [](unsigned char c) {
            return static_cast<char>(std::tolower(c));
        });
        if (!m.empty() && m != "auto") {
            if (m == "irf_convolution" || m == "irf") {
                hydro_system->SetExcitationMethod(ExcitationMethod::kIrfConvolution);
                seastack::infra::cli::LogDebug(seastack::infra::cli::CreateAlignedLine(
                    "•", "Excitation Method", "IRF convolution (YAML override)"));
            } else if (m == "frequency_domain" || m == "fd") {
                hydro_system->SetExcitationMethod(ExcitationMethod::kFrequencyDomain);
                seastack::infra::cli::LogDebug(seastack::infra::cli::CreateAlignedLine(
                    "•", "Excitation Method", "Frequency domain (YAML override)"));
            } else {
                seastack::infra::cli::LogWarning(
                    std::string("Unknown hydrodynamics.excitation.method '") +
                    hydro_data.excitation_method + "'; using auto selection.");
            }
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Convolution truncation (excitation + radiation)
    // ─────────────────────────────────────────────────────────────────────────
    if (hydro_data.excitation_truncation_time > 0.0) {
        hydro_system->SetExcitationTruncationTime(hydro_data.excitation_truncation_time);
        seastack::infra::cli::LogDebug(seastack::infra::cli::CreateAlignedLine(
            "•", "Excitation Truncation", std::to_string(hydro_data.excitation_truncation_time) + "s"));
    }
    if (hydro_data.radiation_truncation_time > 0.0) {
        hydro_system->SetRadiationTruncationTime(hydro_data.radiation_truncation_time);
        seastack::infra::cli::LogDebug(seastack::infra::cli::CreateAlignedLine(
            "•", "Radiation Truncation", std::to_string(hydro_data.radiation_truncation_time) + "s"));
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Radiation method selection
    // ─────────────────────────────────────────────────────────────────────────
    std::string method = hydro_data.radiation_method;
    std::transform(method.begin(), method.end(), method.begin(), ::tolower);
    
    if (method == "state_space") {
        hydro_system->SetRadiationMethod(seastack::hydro::RadiationMethod::kStateSpace);
        seastack::infra::cli::LogDebug(seastack::infra::cli::CreateAlignedLine("•", "Radiation Method", "StateSpace"));
        
        seastack::hydro::StateSpaceOptions ss_opts;
        ss_opts.max_order = hydro_data.ss_max_order;
        ss_opts.r2_threshold = hydro_data.ss_r2_threshold;
        ss_opts.max_hankel_size = hydro_data.ss_max_hankel_size;
        ss_opts.r2_num_samples = hydro_data.ss_r2_num_samples;
        hydro_system->SetStateSpaceOptions(ss_opts);
        
        if (hydro_data.output_kernel_fit) {
            hydro_system->SetOutputKernelFit(true);
            seastack::infra::cli::LogDebug(seastack::infra::cli::CreateAlignedLine("•", "Kernel Fit Diagnostics", "Enabled"));
        }
        
        seastack::infra::cli::LogDebug(seastack::infra::cli::CreateAlignedLine("•", "SS Max Order", std::to_string(ss_opts.max_order)));
        seastack::infra::cli::LogDebug(seastack::infra::cli::CreateAlignedLine("•", "SS R² Threshold", std::to_string(ss_opts.r2_threshold)));
        seastack::infra::cli::LogDebug(seastack::infra::cli::CreateAlignedLine("•", "SS Max Hankel Size", std::to_string(ss_opts.max_hankel_size)));
        seastack::infra::cli::LogDebug(seastack::infra::cli::CreateAlignedLine("•", "SS R² Samples", std::to_string(ss_opts.r2_num_samples)));
    } else {
        hydro_system->SetRadiationMethod(seastack::hydro::RadiationMethod::kRirfConvolution);
        seastack::infra::debug::LogDebug("Radiation method: RIRF Convolution (default)");
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Radiation kernel processing (smoothing + tapering, composable)
    // ─────────────────────────────────────────────────────────────────────────
    auto kp = hydro_data.radiation_kernel_processing;
    kp.smoothing_window = std::max(3, kp.smoothing_window);
    if (kp.smoothing_window % 2 == 0) kp.smoothing_window += 1;  // enforce odd

    if (kp.RequiresProcessing()) {
        hydro_system->SetRadiationKernelProcessing(kp);
        if (kp.smoothing_type != "none") {
            seastack::infra::cli::LogDebug(seastack::infra::cli::CreateAlignedLine("•", "RIRF Smoothing", kp.smoothing_type));
        }
        if (kp.taper_enabled) {
            seastack::infra::cli::LogDebug(seastack::infra::cli::CreateAlignedLine("•", "RIRF Taper", "enabled"));
            seastack::infra::cli::LogDebug(seastack::infra::cli::CreateAlignedLine("•", "Taper Start %", std::to_string(kp.taper_start_fraction)));
            seastack::infra::cli::LogDebug(seastack::infra::cli::CreateAlignedLine("•", "Taper End %", std::to_string(kp.taper_end_fraction)));
            seastack::infra::cli::LogDebug(seastack::infra::cli::CreateAlignedLine("•", "Taper Final Amp", std::to_string(kp.taper_final_amplitude)));
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // MoorDyn mooring coupling (optional)
    // ─────────────────────────────────────────────────────────────────────────
#ifdef SEASTACK_HAVE_MOORDYN
    if (hydro_data.moordyn_enabled && !hydro_data.moordyn_input_file.empty()) {
        MoorDynConfig md_cfg;
        md_cfg.enabled = true;
        md_cfg.input_file = hydro_data.moordyn_input_file;

        // Resolve each MoorDyn body name to a 0-based index in the concatenated
        // list [hydro bodies..., auxiliary bodies...]. Hydrodynamic bodies keep
        // their index in matched_bodies. Any other name is resolved against the
        // full Chrono body list and registered as an auxiliary (mooring-only)
        // coupled body, so MoorDyn can couple e.g. a vehicle chassis that has no
        // BEM data. Auxiliary indices follow the hydro bodies, in registration
        // order.
        int aux_count = 0;
        for (const auto& bname : hydro_data.moordyn_body_names) {
            // 1. Hydrodynamic body?
            int hydro_index = -1;
            for (size_t i = 0; i < matched_bodies.size(); ++i) {
                if (matched_bodies[i]->GetName() == bname) {
                    hydro_index = static_cast<int>(i);
                    break;
                }
            }
            if (hydro_index >= 0) {
                md_cfg.coupled_body_indices.push_back(hydro_index);
                continue;
            }

            // 2. Any other Chrono body -> auxiliary coupled body.
            std::shared_ptr<ChBody> aux_body;
            for (const auto& b : bodies) {
                if (b->GetName() == bname) {
                    aux_body = b;
                    break;
                }
            }
            if (aux_body) {
                hydro_system->AddAuxiliaryCoupledBody(aux_body);
                md_cfg.coupled_body_indices.push_back(
                    static_cast<int>(matched_bodies.size()) + aux_count);
                ++aux_count;
                continue;
            }

            // 3. Not found anywhere: report with the available names.
            std::ostringstream available;
            for (const auto& b : bodies) {
                available << " '" << b->GetName() << "'";
            }
            throw std::runtime_error(
                "MoorDyn config references body '" + bname +
                "' which was not found among the hydrodynamic bodies or the "
                "Chrono system bodies. Available Chrono bodies:" + available.str());
        }

        hydro_system->SetMoorDynConfig(md_cfg);
        seastack::infra::cli::LogDebug(seastack::infra::cli::CreateAlignedLine(
            "+", "MoorDyn", hydro_data.moordyn_input_file +
            " (" + std::to_string(md_cfg.coupled_body_indices.size()) + " bodies, " +
            std::to_string(aux_count) + " auxiliary)"));
    }
#else
    if (hydro_data.moordyn_enabled) {
        seastack::infra::cli::LogWarning(
            "MoorDyn is enabled in YAML but SEA-Stack was built without "
            "SEASTACK_ENABLE_MOORDYN. Mooring forces will be ignored.");
    }
#endif

    // ─────────────────────────────────────────────────────────────────────────
    // Per-body damping (linear + optional quadratic)
    // ─────────────────────────────────────────────────────────────────────────
    {
        auto extract_damping = [&](auto member_ptr, const char* label, auto setter) {
            std::vector<std::array<double, 6>> per_body;
            per_body.reserve(hydro_data.bodies.size());
            bool has_any = false;
            for (const auto& body : hydro_data.bodies) {
                per_body.push_back(body.*member_ptr);
                for (double c : body.*member_ptr) {
                    if (c != 0.0) { has_any = true; break; }
                }
            }
            if (has_any) {
                setter(per_body);
                for (size_t bi = 0; bi < hydro_data.bodies.size(); ++bi) {
                    bool body_has = false;
                    for (double c : hydro_data.bodies[bi].*member_ptr) {
                        if (c != 0.0) { body_has = true; break; }
                    }
                    if (body_has) {
                        std::ostringstream oss;
                        oss << "[";
                        for (int i = 0; i < 6; ++i) {
                            if (i > 0) oss << ", ";
                            oss << (hydro_data.bodies[bi].*member_ptr)[i];
                        }
                        oss << "]";
                        seastack::infra::cli::LogDebug(seastack::infra::cli::CreateAlignedLine(
                            "+", std::string(label) + " (" + hydro_data.bodies[bi].name + ")", oss.str()));
                    }
                }
            }
        };

        extract_damping(&seastack::hydro::HydroBody::linear_damping,
                        "Linear Damping",
                        [&](const auto& v) { hydro_system->SetLinearDamping(v); });
        extract_damping(&seastack::hydro::HydroBody::quadratic_damping,
                        "Quadratic Damping",
                        [&](const auto& v) { hydro_system->SetQuadraticDamping(v); });
    }

    return hydro_system;
}

// ------------------------------------------------------------
// SECTION: Convenience loader + setup helper
// ------------------------------------------------------------
// Keeps file parsing and setup logic together for callers that
// only know the YAML path.

std::unique_ptr<HydroSystem> SetupHydroFromYAMLFile(
    const std::string& hydro_yaml_path,
    const std::vector<std::shared_ptr<ChBody>>& bodies,
    double timestep,
    double sim_duration,
    double ramp_duration) {
    const YAMLHydroData hydro_data = seastack::hydro::LoadHydroConfigFromYaml(hydro_yaml_path);
    return SetupHydroFromYAML(hydro_data, bodies, timestep, sim_duration, ramp_duration);
}

}  // namespace seastack::chrono
