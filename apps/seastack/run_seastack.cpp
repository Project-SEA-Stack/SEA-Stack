/**
 * @file run_seastack.cpp
 * @brief CLI entrypoint for the SEA-Stack YAML-based runner.
 */

#include "run_from_yaml.h"
#include "campaign_runner.h"
#include "single_run.h"
#include "cell_io.h"
#include "app_init.h"
#include "doctor.h"
#include <seastack/config.h>
#include <seastack/version.h>
#include "misc_options.h"
#include <seastack/infra/logging.h>
#include <string>
#include <filesystem>
#include <iostream>

#ifdef _WIN32
#include <windows.h>
#endif

namespace {

static void PrintBanner() noexcept { seastack::infra::cli::ShowBanner(); }

static void PrintVersion() noexcept { seastack::infra::cli::LogInfo(std::string(SEASTACK_NAME) + " version " + SEASTACK_VERSION); }

static void PrintInfo() noexcept { seastack::infra::cli::ShowBanner(); }

void PrintHelp(const char* program_name) {
    seastack::infra::cli::ShowEmptyLine();
    seastack::infra::cli::LogInfo("USAGE");
    seastack::infra::cli::LogInfo(std::string("  ") + program_name + " [options] <input_directory>");
    seastack::infra::cli::LogInfo(std::string("  ") + program_name + " [options] <model.setup.yaml>");
    seastack::infra::cli::LogInfo(std::string("  ") + program_name + " --campaign <campaign.yaml>");
    seastack::infra::cli::ShowEmptyLine();
    seastack::infra::cli::LogInfo("MODES");
    seastack::infra::cli::LogInfo("  Single run (default):  run one simulation from a case directory");
    seastack::infra::cli::LogInfo("  Power matrix:          generate a power matrix from a campaign YAML");
    seastack::infra::cli::ShowEmptyLine();
    seastack::infra::cli::LogInfo("OPTIONS");
    seastack::infra::cli::LogInfo("  -h, --help           Show this help message and exit");
    seastack::infra::cli::LogInfo("  -v, --version        Show version and exit");
    seastack::infra::cli::LogInfo("  -i, --info           Print project and license info");
    seastack::infra::cli::LogInfo("      --doctor         Run environment/package diagnostics and exit");
    seastack::infra::cli::LogInfo("      --campaign FILE  Generate a power matrix from a campaign YAML");
    seastack::infra::cli::LogInfo("      --nogui          Disable GUI visualization");
    seastack::infra::cli::LogInfo("      --log            Enable detailed logging to file");
    seastack::infra::cli::LogInfo("      --model_file     Override model YAML file (default: auto-detected)");
    seastack::infra::cli::LogInfo("      --sim_file       Override simulation YAML file (default: auto-detected)");
    seastack::infra::cli::LogInfo("      --nobanner       Disable banner display");
    seastack::infra::cli::LogInfo("      --quiet          Quiet mode (warnings/errors and a one-line success summary)");
    seastack::infra::cli::LogInfo("      --debug          Enable detailed simulation diagnostics");
    seastack::infra::cli::LogInfo("      --trace          Enable step-by-step simulation tracing (implies --debug)");
    seastack::infra::cli::LogInfo("      --output-level L Override HDF5 output level (compact|standard|detailed)");
    seastack::infra::cli::ShowEmptyLine();
    seastack::infra::cli::LogInfo("EXAMPLES");
    seastack::infra::cli::LogInfo(std::string("  # Run simulation with GUI using directory"));
    seastack::infra::cli::LogInfo(std::string("  ") + program_name + " ./cases/rm3/");
    seastack::infra::cli::ShowEmptyLine();
    seastack::infra::cli::LogInfo(std::string("  # Run simulation using setup file directly"));
    seastack::infra::cli::LogInfo(std::string("  ") + program_name + " ./cases/rm3/model.setup.yaml");
    seastack::infra::cli::ShowEmptyLine();
    seastack::infra::cli::LogInfo(std::string("  # Run simulation without GUI (headless mode)"));
    seastack::infra::cli::LogInfo(std::string("  ") + program_name + " ./my_case/ --nogui");
    seastack::infra::cli::ShowEmptyLine();
    seastack::infra::cli::LogInfo(std::string("  # Override YAML files"));
    seastack::infra::cli::LogInfo(std::string("  ") + program_name + " ./ --model_file alt.model.yaml --sim_file alt.sim.yaml");
    seastack::infra::cli::ShowEmptyLine();
    seastack::infra::cli::LogInfo("INPUT DIRECTORY");
    seastack::infra::cli::LogInfo("  Directory containing *.setup.yaml or individual YAML files:");
    seastack::infra::cli::ShowEmptyLine();
    seastack::infra::cli::LogInfo("  - *.setup.yaml         (optional, recommended)");
    seastack::infra::cli::LogInfo("    → defines model/simulation/hydro/output files");
    seastack::infra::cli::ShowEmptyLine();
    seastack::infra::cli::LogInfo("  - *.model.yaml         (required if no setup file)");
    seastack::infra::cli::LogInfo("  - *.simulation.yaml    (required if no setup file)");
    seastack::infra::cli::ShowEmptyLine();
    seastack::infra::cli::LogInfo("EXIT CODES");
    seastack::infra::cli::LogInfo("  0  Success");
    seastack::infra::cli::LogInfo("  1  Setup or configuration error");
    seastack::infra::cli::LogInfo("  2  Simulation diverged (single run)");
    seastack::infra::cli::LogInfo("  3  Campaign completed with one or more failed cells");
    seastack::infra::cli::ShowEmptyLine();
}

struct CLIArgs {
    std::string input_directory;
    std::string model_file;
    std::string sim_file;
    std::string campaign_file;          // --campaign <file>
    std::string run_cell_file;          // --run-cell <file> (subprocess IPC)
    bool nogui = false;
    bool log = false;
    bool nobanner = false;
    bool quiet = false;
    bool debug = false;
    bool trace = false;
    std::string output_level;
    bool profile = false;
};

static CLIArgs ParseArguments(int argc, char* argv[]) {
    CLIArgs args;
    
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        
        if (arg == "--nogui") {
            args.nogui = true;
        } else if (arg == "--log" || arg == "--logging") {
            args.log = true;
        } else if (arg == "--nobanner") {
            args.nobanner = true;
        } else if (arg == "--quiet") {
            args.quiet = true;
        } else if (arg == "--debug") {
            args.debug = true;
        } else if (arg == "--trace") {
            args.trace = true;
            args.debug = true;  // trace implies debug
        } else if (arg == "--model_file") {
            if (i + 1 < argc) {
                args.model_file = argv[++i];
            } else {
                seastack::infra::cli::LogError("ERROR: --model_file requires a file path argument");
                std::exit(1);
            }
        } else if (arg == "--sim_file") {
            if (i + 1 < argc) {
                args.sim_file = argv[++i];
            } else {
                seastack::infra::cli::LogError("ERROR: --sim_file requires a file path argument");
                std::exit(1);
            }
        } else if (arg == "--campaign") {
            if (i + 1 < argc) {
                args.campaign_file = argv[++i];
            } else {
                seastack::infra::cli::LogError("ERROR: --campaign requires a campaign YAML file path");
                std::exit(1);
            }
        } else if (arg == "--run-cell") {
            if (i + 1 < argc) {
                args.run_cell_file = argv[++i];
            } else {
                seastack::infra::cli::LogError("ERROR: --run-cell requires a cell config YAML path");
                std::exit(1);
            }
        } else if (arg == "--output-level") {
            if (i + 1 < argc) {
                args.output_level = argv[++i];
            } else {
                seastack::infra::cli::LogError("ERROR: --output-level requires a value (compact|standard|detailed)");
                std::exit(1);
            }
        } else if (arg == "--doctor") {
            // Already handled in early flag scan; ignore here.
        } else if (arg == "--profile") {
            args.profile = true;
        } else if (arg.substr(0, 1) != "-") {
            // This is a positional argument (input directory)
            if (args.input_directory.empty()) {
                args.input_directory = arg;
            } else {
                seastack::infra::cli::LogError("ERROR: Multiple input directories specified. Only one is allowed.");
                std::exit(1);
            }
        } else {
            seastack::infra::cli::LogError(std::string("ERROR: Unknown option: ") + arg);
            seastack::infra::cli::LogInfo("Use --help for usage information.");
            std::exit(1);
        }
    }
    
    return args;
}

} // anonymous namespace

int main(int argc, char* argv[]) {
    // ---------------------------------------------------------------------
    // Configure UTF-8 console output on Windows (must be first!)
    // ---------------------------------------------------------------------
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
    // Enable ANSI escape codes (e.g. colors) on Windows 10+
    {
        HANDLE hOut = GetStdHandle(STD_OUTPUT_HANDLE);
        if (hOut != INVALID_HANDLE_VALUE) {
            DWORD mode = 0;
            if (GetConsoleMode(hOut, &mode)) {
                SetConsoleMode(hOut, mode | 0x0004);  // ENABLE_VIRTUAL_TERMINAL_PROCESSING
            }
        }
    }
#endif

    // Check for hidden options first (before any other processing)
    if (seastack::app::HandleHiddenOptions(argc, argv)) {
        return 0;
    }
    
    // -------------------------------------------------------------------------
    // Initialize logging early so all CLI output uses the nice formatting
    // Wrapped in try/catch to report initialization failures clearly
    // -------------------------------------------------------------------------
    try {
        seastack::infra::LoggingConfig cfg;
        cfg.enable_cli_output = true;
        cfg.enable_file_output = false;
        cfg.console_level = seastack::infra::LogLevel::Info;
        cfg.file_level = seastack::infra::LogLevel::Info;
        (void)seastack::infra::Initialize(cfg);  // Ignore return value; failures throw
    } catch (const std::exception& e) {
        std::cerr << "FATAL: Exception during logging initialization: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "FATAL: Unknown exception during logging initialization" << std::endl;
        return 1;
    }

    // Check for help/version/info/doctor flags first (before requiring input directory)
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            PrintHelp(argv[0]);
            seastack::infra::Shutdown();
            return 0;
        } else if (arg == "--version" || arg == "-v") {
            PrintVersion();
            seastack::infra::Shutdown();
            return 0;
        } else if (arg == "--info" || arg == "-i") {
            PrintInfo();
            seastack::infra::Shutdown();
            return 0;
        } else if (arg == "--doctor") {
            int result = seastack::app::RunDoctor(argv[0]);
            seastack::infra::Shutdown();
            return result;
        }
    }

    // Handle "no arguments" case
    if (argc == 1) {
        seastack::infra::cli::LogError("ERROR: Input directory or setup file is required");
        seastack::infra::cli::ShowEmptyLine();
        seastack::infra::cli::LogInfo(std::string("Usage: ") + argv[0] + " [options] <input_directory_or_setup_file>");
        seastack::infra::cli::LogInfo("Use --help for more information.");
        seastack::infra::Shutdown();
        return 1;
    }
    
    // Parse command line arguments
    CLIArgs args = ParseArguments(argc, argv);

    // ---- Subprocess cell mode: --run-cell <file> ----
    if (!args.run_cell_file.empty()) {
        {
            seastack::infra::LoggingConfig lc;
            lc.enable_cli_output = !args.quiet;
            lc.enable_file_output = false;
            lc.enable_debug_logging = args.debug;
            lc.console_level = args.debug ? seastack::infra::LogLevel::Debug
                                          : seastack::infra::LogLevel::Info;
            lc.file_level = seastack::infra::LogLevel::Info;
            seastack::infra::UpdateLoggingConfig(lc);
        }
        seastack::app::InitChronoEnvironment();
        try {
            auto cell_cfg = seastack::app::ReadCellConfigYAML(args.run_cell_file);
            auto result = seastack::app::RunSingleCase(cell_cfg);
            std::string result_path = args.run_cell_file + ".result.yaml";
            seastack::app::WriteCellResultYAML(result_path, result);
            seastack::infra::Shutdown();
            return result.exit_code;
        } catch (const std::exception& e) {
            std::cerr << "run-cell fatal: " << e.what() << std::endl;
            seastack::infra::Shutdown();
            return 1;
        }
    }

    // ---- Campaign mode: --campaign <file> ----
    if (!args.campaign_file.empty()) {
        std::filesystem::path cpath(args.campaign_file);
        if (!std::filesystem::exists(cpath)) {
            seastack::infra::cli::LogError("ERROR: Campaign file does not exist: " + args.campaign_file);
            seastack::infra::Shutdown();
            return 1;
        }

        {
            seastack::infra::LoggingConfig lc;
            lc.enable_cli_output = !args.quiet;
            lc.enable_file_output = false;
            lc.enable_debug_logging = args.debug;
            lc.console_level = args.debug ? seastack::infra::LogLevel::Debug
                                          : seastack::infra::LogLevel::Info;
            lc.file_level = seastack::infra::LogLevel::Info;
            seastack::infra::UpdateLoggingConfig(lc);
        }

        if (!args.quiet && !args.nobanner) {
            seastack::infra::cli::ShowBannerCompact();
        }
        seastack::infra::debug::LogDebug(std::string("[startup] SEA-Stack v") + SEASTACK_VERSION +
            " — power matrix mode");

        seastack::app::InitChronoEnvironment();

        int ret = seastack::app::RunCampaign(args.campaign_file, args.debug, args.profile, args.quiet);
        seastack::infra::Shutdown();
        return ret;
    }

    // Validate required input directory or setup file
    if (args.input_directory.empty()) {
        seastack::infra::cli::LogError("ERROR: Input directory or setup file is required");
        seastack::infra::cli::ShowEmptyLine();
        seastack::infra::cli::LogInfo(std::string("Usage: ") + argv[0] + " [options] <input_directory_or_setup_file>");
        seastack::infra::cli::LogInfo("Use --help for more information.");
        seastack::infra::Shutdown();
        return 1;
    }
    
    // Check if input is a setup file or directory
    std::filesystem::path input_path(args.input_directory);
    if (std::filesystem::exists(input_path)) {
        if (std::filesystem::is_regular_file(input_path)) {
            if (input_path.extension() == ".yaml") {
                const std::string filename = input_path.filename().string();
                const std::string suffix = ".setup.yaml";
                if (filename.length() >= suffix.length() && 
                    filename.compare(filename.length() - suffix.length(), suffix.length(), suffix) == 0) {
                    args.input_directory = input_path.parent_path().string();
                    seastack::infra::cli::LogInfo(std::string("Loaded setup file: ") + input_path.string());
                } else {
                    seastack::infra::cli::LogError("ERROR: File provided is not a valid .setup.yaml file");
                    seastack::infra::cli::LogInfo(std::string("  Path: ") + args.input_directory);
                    seastack::infra::cli::LogInfo("  Expected: Directory or any file ending in '.setup.yaml'");
                    seastack::infra::Shutdown();
                    return 1;
                }
            }
        } else if (!std::filesystem::is_directory(input_path)) {
            seastack::infra::cli::LogError("ERROR: Path is neither a directory nor a regular file");
            seastack::infra::cli::LogInfo(std::string("  Path: ") + args.input_directory);
            seastack::infra::Shutdown();
            return 1;
        }
    } else {
        seastack::infra::cli::LogError("ERROR: Input path does not exist");
        seastack::infra::cli::LogInfo(std::string("  Path: ") + args.input_directory);
        seastack::infra::Shutdown();
        return 1;
    }
    
    // Shutdown logging - the runner will reinitialize it
    seastack::infra::Shutdown();
    
    // Prepare arguments for the YAML runner
    std::vector<std::string> runner_args;
    runner_args.push_back(argv[0]);
    runner_args.push_back(args.input_directory);
    
    if (args.nogui) runner_args.push_back("--nogui");
    if (args.log) runner_args.push_back("--log");
    if (args.nobanner) runner_args.push_back("--nobanner");
    if (args.quiet) runner_args.push_back("--quiet");
    if (args.debug) runner_args.push_back("--debug");
    if (args.trace) runner_args.push_back("--trace");
    if (args.profile) runner_args.push_back("--profile");
    if (!args.model_file.empty()) {
        runner_args.push_back("--model_file");
        runner_args.push_back(args.model_file);
    }
    if (!args.sim_file.empty()) {
        runner_args.push_back("--sim_file");
        runner_args.push_back(args.sim_file);
    }
    if (!args.output_level.empty()) {
        runner_args.push_back("--output-level");
        runner_args.push_back(args.output_level);
    }
    
    // Convert to argc/argv format for the runner
    std::vector<char*> runner_argv;
    for (const auto& arg : runner_args) {
        runner_argv.push_back(const_cast<char*>(arg.c_str()));
    }
    
    // Call the YAML runner
    int ret = seastack::app::RunFromYAML(static_cast<int>(runner_argv.size()), runner_argv.data());
    if (ret != 0) {
        std::cerr << "\nSimulation failed (exit code " << ret << ").\n"
                  << "  - Run with --doctor to diagnose environment/package issues.\n"
                  << "  - Run with --debug  for detailed diagnostic logs.\n"
                  << "  - Run with --nogui  if visualization is unavailable.\n"
                  << std::endl;
    }
    return ret;
}
