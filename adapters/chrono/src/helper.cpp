/*********************************************************************
 * @file  helper.cpp
 *
 * @brief Implementation of helper utilities.
 *********************************************************************/

#include <seastack/config.h>
#include <seastack/adapters/chrono/helper.h>
#include <seastack/infra/logging.h>

#include <chrono/core/ChDataPath.h>
#include <chrono_thirdparty/cxxopts/ChCLI.h>

#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

bool seastack::chrono::GetCLIArguments(int argc,
                             char** argv,
                             const std::string& description,
                             bool& output,
                             bool& profile,
                             bool& plot,
                             bool& gui,
                             std::string& data_dir) {
    ::chrono::ChCLI cli(argv[0], description);

    cli.AddOption<std::string>("", "data_dir", "SEA-Stack data directory", "");

    if (output)
        cli.AddOption<bool>("", "no_output", "Disable generation of simulation output");
    else
        cli.AddOption<bool>("", "output", "Enable generation of simulation output");

    if (profile)
        cli.AddOption<bool>("", "no_profile", "Disable profiling of simulation time");
    else
        cli.AddOption<bool>("", "profile", "Enable profiling of simulation time");

    if (plot)
        cli.AddOption<bool>("", "no_plot", "Disable final plotting");
    else
        cli.AddOption<bool>("", "plot", "Enable final plotting");

    if (gui)
        cli.AddOption<bool>("", "no_gui", "Disable GUI");
    else
        cli.AddOption<bool>("", "gui", "Enable GUI");

    if (!cli.Parse(argc, argv)) {
        cli.Help();
        return false;
    }

    data_dir = cli.Get("data_dir").as<std::string>();

    if (output)
        output = !cli.GetAsType<bool>("no_output");
    else
        output = cli.GetAsType<bool>("output");

    if (profile)
        profile = !cli.GetAsType<bool>("no_profile");
    else
        profile = cli.GetAsType<bool>("profile");

    if (plot)
        plot = !cli.GetAsType<bool>("no_plot");
    else
        plot = cli.GetAsType<bool>("plot");

    if (gui)
        gui = !cli.GetAsType<bool>("no_gui");
    else
        gui = cli.GetAsType<bool>("gui");

    return true;
}

using std::filesystem::path;
using std::filesystem::absolute;

static path DATADIR{};

// Last Chrono data path passed to SetChronoDataPath (for subprocess env export).
static std::string g_chrono_data_dir;

bool seastack::chrono::SetInitialEnvironment(const std::string& data_dir) {
    const char* env_p = std::getenv("SEASTACK_DATA_DIR");

    if (env_p) {
        // Highest priority: explicit environment override
        DATADIR = absolute(path(env_p));
        seastack::infra::debug::LogDebug(std::string("Using data directory from SEASTACK_DATA_DIR: '") + GetDataDir() + "'");
    } else if (!data_dir.empty()) {
        DATADIR = absolute(path(data_dir));
        seastack::infra::debug::LogDebug(std::string("Using provided data directory: '") + GetDataDir() + "'");
    } else {
        // Try exe-relative path for packaged install (e.g. bin/../data)
        std::string exe_dir = seastack::infra::GetExecutableDirectory();
        path exe_relative_data;
        if (!exe_dir.empty()) {
            exe_relative_data = path(exe_dir) / ".." / "data";
            exe_relative_data = exe_relative_data.lexically_normal();
        }
        if (!exe_dir.empty() && std::filesystem::exists(exe_relative_data) && std::filesystem::is_directory(exe_relative_data)) {
            DATADIR = absolute(exe_relative_data);
            seastack::infra::debug::LogDebug(std::string("Using exe-relative data directory: '") + GetDataDir() + "'");
        } else {
            DATADIR = absolute(path(SEASTACK_DATA_DIR_PATH));
            seastack::infra::debug::LogDebug(std::string("Using default data directory SEASTACK_DATA_DIR_PATH: '") + GetDataDir() + "'");
        }
    }

    // Set Chrono data directory.
    // Prefer the canonical Chrono data tree (SEASTACK_CHRONO_DATA_DIR_PATH)
    // when it exists, because Chrono 9's ChColormap has a static-init /
    // double-prefix bug that only resolves correctly against the full upstream
    // data layout (colormaps/ as a direct child of the data root).
    // Fall back to exe-relative or SEASTACK_DATA_DIR_PATH/chrono for packaged
    // installs that ship only the copied subset.
    path chrono_data_path;
#ifdef SEASTACK_CHRONO_DATA_DIR_PATH
    {
        path canonical(SEASTACK_CHRONO_DATA_DIR_PATH);
        if (std::filesystem::exists(canonical) && std::filesystem::is_directory(canonical)) {
            chrono_data_path = canonical;
        }
    }
#endif
    if (chrono_data_path.empty()) {
        std::string exe_dir = seastack::infra::GetExecutableDirectory();
        if (!exe_dir.empty()) {
            path exe_rel = path(exe_dir) / ".." / "data" / "chrono";
            exe_rel = exe_rel.lexically_normal();
            if (std::filesystem::exists(exe_rel) && std::filesystem::is_directory(exe_rel)) {
                chrono_data_path = exe_rel;
            }
        }
    }
    if (chrono_data_path.empty()) {
        path seastack_chrono(std::string(SEASTACK_DATA_DIR_PATH) + "/chrono");
        if (std::filesystem::exists(seastack_chrono) && std::filesystem::is_directory(seastack_chrono)) {
            chrono_data_path = seastack_chrono;
        }
    }
    if (!chrono_data_path.empty()) {
        std::string chrono_str = chrono_data_path.generic_string();
        if (!chrono_str.empty() && chrono_str.back() != '/') chrono_str.push_back('/');
        g_chrono_data_dir = chrono_str;
        ::chrono::SetChronoDataPath(chrono_str);
    } else {
        g_chrono_data_dir.clear();
    }

    return true;
}

std::string seastack::chrono::GetChronoDataDir() {
    return g_chrono_data_dir;
}

double seastack::chrono::GetSimDuration(double short_duration, double long_duration) {
    const char* env = std::getenv("SEASTACK_LONG_TESTS");
    if (env) {
        std::string val(env);
        if (val == "1" || val == "true" || val == "TRUE" || val == "ON")
            return long_duration;
    }
    return short_duration;
}

std::string seastack::chrono::GetDataDir() {
    return DATADIR.lexically_normal().generic_string();
}

void seastack::chrono::EnsureDirectoryExists(const std::filesystem::path& path) {
    if (!std::filesystem::exists(path)) {
        std::cout << "Path " << std::filesystem::absolute(path) << " does not exist, creating it now..." << std::endl;
        std::filesystem::create_directory(path);
    }
}

std::string seastack::chrono::GetDemoOutDir() {
    path base = seastack::infra::GetExecutableDirectory().empty()
                    ? std::filesystem::current_path()
                    : path(seastack::infra::GetExecutableDirectory());
    path dir_path = base / "results" / "demos";
    std::filesystem::create_directories(dir_path);
    return dir_path.lexically_normal().generic_string();
}

std::string seastack::chrono::GetTestOutDir() {
    path base = seastack::infra::GetExecutableDirectory().empty()
                    ? std::filesystem::current_path()
                    : path(seastack::infra::GetExecutableDirectory());
    path dir_path = base / "results" / "tests";
    std::filesystem::create_directories(dir_path);
    return dir_path.lexically_normal().generic_string();
}
