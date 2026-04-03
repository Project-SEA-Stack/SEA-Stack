#ifndef SEASTACK_ADAPTERS_CHRONO_HELPER_H
#define SEASTACK_ADAPTERS_CHRONO_HELPER_H

#include <fstream>
#include <iostream>
#include <string>
#include <filesystem>  // C++17

#include <Eigen/Dense>  // Need for the container function

#include <seastack/infra/logging.h>
#include <seastack/core/math_constants.h>

/**@brief Chrono adapter utilities for SEA-Stack library
 *
 */
namespace seastack::chrono {

/**@brief Get program command line arguments
 * @param argc     - number of argument (same as for main function)
 * @param argv     - arguments of main function
 * @output         - true if generating output file(s)
 * @profile        - true is saving timing information
 * @plot           - initial default for --plot/--no_plot; demos may ignore (reports use matplotlib)
 * @param gui      - true if using run-time visualization
 * @param data_dir - alternative SEA-Stack data directory
 * @return false on error and true otherwise
 */
bool GetCLIArguments(int argc,
                     char** argv,
                     const std::string& description,
                     bool& output,
                     bool& profile,
                     bool& plot,
                     bool& gui,
                     std::string& data_dir);

/**@brief Set initial environment
 *
 * Set the main SEA-Stack data directory and the Chrono data directory.
 * The main SEA-Stack data directory is set, in order, using:
 * - the environment variable SEASTACK_DATA_DIR (if defined)
 * - the provided data_dir path (if non-empty)
 * - the default data directory in the SEA-Stack source tree
 *
 * @param data_dir alternative SEA-Stack data directory
 * @return false on error and true otherwise
 */
bool SetInitialEnvironment(const std::string& data_dir);

/**@brief Chrono data directory last applied by SetInitialEnvironment
 *
 * Normalized path with trailing slash (same string passed to Chrono's
 * SetChronoDataPath), or empty if no Chrono data root was resolved.
 */
std::string GetChronoDataDir();

/**@brief Get simulation duration based on SEASTACK_LONG_TESTS environment variable.
 *
 * Tests use shorter durations by default for fast CI/regression checks.
 * Set SEASTACK_LONG_TESTS=1 to use longer durations for publication-quality results.
 *
 * @param short_duration Duration (seconds) for quick testing
 * @param long_duration  Duration (seconds) for thorough/publication testing
 * @return short_duration normally, long_duration when SEASTACK_LONG_TESTS=1
 */
double GetSimDuration(double short_duration, double long_duration);

/**@brief Get base name of data directory
 *
 * @return the string containing the path in standard format
 */
std::string GetDataDir();

/**@brief C++ 17 filesystem helper to ensure a directory exists
 *
 */
void EnsureDirectoryExists(const std::filesystem::path& path);

std::string GetDemoOutDir();
std::string GetTestOutDir();

template <typename T>
void WriteDataToFile(const std::vector<T>& data, const std::string& filename) {
    std::ofstream outFile(filename);
    if (outFile.is_open()) {
        for (const auto& item : data) {
            outFile << item << std::endl;
        }
        outFile.close();
    } else {
        seastack::infra::cli::LogError(std::string("Unable to open the file for writing: ") + filename);
    }
};

/**
 * @brief Prints contents of 1D Container data to given file.
 *
 * @param container the 1D array/vector to write to file
 * @param file_name file to write container to
 */
template <typename Container>
void WriteContainerToFile(const Container& container, const std::string& file_name);

/**
 * @brief Prints contents of std::vector<double> data to given file.
 *
 * @param container std::vector<double> to write to file
 * @param file_name file to write container to
 */
template <>
void inline WriteContainerToFile<std::vector<double>>(const std::vector<double>& container,
                                                      const std::string& file_name) {
    std::ofstream output_file(file_name);

    if (!output_file) {
        seastack::infra::cli::LogError(std::string("Error: Unable to open the file: ") + file_name);
        return;
    }

    for (const double value : container) {
        output_file << value << std::endl;
    }

    output_file.close();
};

/**
 * @brief Prints contents of Eigen::VectorXd data to given file.
 *
 * @param container Eigen::VectorXd to write to file
 * @param file_name file to write container to
 */
template <>
void inline WriteContainerToFile<Eigen::VectorXd>(const Eigen::VectorXd& container, const std::string& file_name) {
    std::ofstream output_file(file_name);

    if (!output_file) {
        seastack::infra::cli::LogError(std::string("Error: Unable to open the file: ") + file_name);
        return;
    }

    for (int i = 0; i < container.size(); ++i) {
        output_file << container[i] << std::endl;
    }

    output_file.close();
};

}  // end namespace seastack::chrono

#endif