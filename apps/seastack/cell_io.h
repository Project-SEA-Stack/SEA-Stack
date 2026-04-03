/*********************************************************************
 * @file  cell_io.h
 * @brief YAML-based IPC for subprocess cell execution.
 *
 * Provides serialization/deserialization of SingleRunConfig and
 * SingleRunResult for the --run-cell subprocess mode.
 *********************************************************************/

#ifndef SEASTACK_APP_CELL_IO_H
#define SEASTACK_APP_CELL_IO_H

#include "single_run.h"
#include <string>

namespace seastack::app {

/// Write a SingleRunConfig as a YAML cell-config file.
/// Used by the campaign runner to prepare subprocess inputs.
void WriteCellConfigYAML(const std::string& path,
                         const SingleRunConfig& config,
                         double hs, double tp, double heading_deg, int seed);

/// Read a cell config YAML and return a populated SingleRunConfig.
/// Also fills wave override fields (hs/tp/heading/seed) into the config's
/// hydro_source variant.
SingleRunConfig ReadCellConfigYAML(const std::string& path);

/// Write a SingleRunResult as a YAML result file (sidecar output).
void WriteCellResultYAML(const std::string& path,
                         const SingleRunResult& result);

/// Read a SingleRunResult from a YAML sidecar file.
SingleRunResult ReadCellResultYAML(const std::string& path);

}  // namespace seastack::app

#endif  // SEASTACK_APP_CELL_IO_H
