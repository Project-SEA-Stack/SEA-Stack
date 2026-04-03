#!/usr/bin/env python3
"""
SEA-Stack Verification Data Normalization Utilities

Shared helpers for converting raw external reference data into the
standardized SEA-Stack verification format.

Standard format rules:
  - Header lines start with '#'
  - Required metadata: format, source, model, quantity, units, columns
  - Space-delimited numeric columns
  - Time in column 0 (seconds)
  - SI units throughout (m, rad, N, s)
  - No tabs

Usage:
    from normalize_utils import write_timeseries, write_rao, write_manifest
"""

import json
import os
from datetime import datetime
from pathlib import Path

import numpy as np


SCHEMA_VERSION = "1.0"


def write_timeseries(path, data, *, source, model, quantity, units, columns,
                     extra_comments=None):
    """Write a time-series file in standard verification format.

    Args:
        path:       Output file path.
        data:       2-D array, column 0 = time.
        source:     Source tool identifier (e.g. "wecsim_moordyn").
        model:      Verification case name (e.g. "rm3_mooring").
        quantity:   What is being stored (e.g. "body_motions").
        units:      Comma-separated unit string (e.g. "s, m, m").
        columns:    Space-separated column header (e.g. "Time(s) HeaveZ(m)").
        extra_comments: Optional list of additional comment lines.
    """
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    header_lines = [
        "# SEA-Stack Verification Data",
        f"# format: timeseries",
        f"# source: {source}",
        f"# model: {model}",
        f"# quantity: {quantity}",
        f"# units: {units}",
        f"# columns: {columns}",
    ]
    if extra_comments:
        for c in extra_comments:
            header_lines.append(f"# {c}")

    header = "\n".join(header_lines)
    np.savetxt(str(path), data, header=header, comments="", fmt="%.6e")


def write_rao(path, data, *, source, model, quantity, units, columns,
              extra_comments=None):
    """Write an RAO file in standard verification format.

    Args:
        path:       Output file path.
        data:       2-D array, column 0 = omega (rad/s).
        source:     Source tool identifier.
        model:      Verification case name.
        quantity:   What is being stored (e.g. "heave_rao").
        units:      Comma-separated unit string (e.g. "rad/s, m/m, rad").
        columns:    Space-separated column header.
        extra_comments: Optional list of additional comment lines.
    """
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    header_lines = [
        "# SEA-Stack Verification Data",
        f"# format: rao",
        f"# source: {source}",
        f"# model: {model}",
        f"# quantity: {quantity}",
        f"# units: {units}",
        f"# columns: {columns}",
    ]
    if extra_comments:
        for c in extra_comments:
            header_lines.append(f"# {c}")

    header = "\n".join(header_lines)
    np.savetxt(str(path), data, header=header, comments="", fmt="%.6e")


def load_standard(path):
    """Load a standard verification data file, returning (metadata, data).

    Returns:
        (metadata_dict, numpy_array)
    """
    metadata = {}
    with open(path, 'r', encoding='utf-8') as f:
        for line in f:
            if not line.startswith('#'):
                break
            if ':' in line:
                key_val = line.lstrip('#').strip()
                key, _, val = key_val.partition(':')
                metadata[key.strip()] = val.strip()

    data = np.loadtxt(str(path), comments='#')
    return metadata, data


# Unit conversions that load_and_normalize() applies automatically.
_UNIT_CONVERSIONS = {
    ('deg', 'rad'): np.pi / 180.0,
    ('deg/m', 'rad/m'): np.pi / 180.0,
}

# Canonical units per CONVENTIONS.md.
_CANONICAL_UNITS = {
    's', 'm', 'rad', 'N', 'm/m', 'rad/m', 'rad/s', '-', 'kg',
}


def load_and_normalize(path):
    """Load a standard verification file and convert to canonical SI units.

    Reads the file via load_standard(), inspects the ``units`` metadata field,
    and applies safe automatic conversions (e.g. deg -> rad).

    Returns:
        (time_array, data_columns_array, metadata_dict)

    ``data_columns_array`` has shape (N, C) where C is the number of *non-time*
    columns.  ``metadata_dict`` is updated so that ``units`` reflects the
    post-conversion canonical units.

    Raises:
        ValueError: if a unit is unrecognised or ambiguous.
    """
    meta, raw = load_standard(path)

    units_str = meta.get('units', '')
    if not units_str:
        raise ValueError(f"No 'units' metadata in {path}")

    units = [u.strip() for u in units_str.split(',')]
    if len(units) != raw.shape[1]:
        raise ValueError(
            f"Unit count ({len(units)}) != column count ({raw.shape[1]}) in {path}"
        )

    canonical = list(units)
    for col_idx, unit in enumerate(units):
        if unit in _CANONICAL_UNITS:
            continue
        converted = False
        for (from_u, to_u), factor in _UNIT_CONVERSIONS.items():
            if unit == from_u:
                raw[:, col_idx] *= factor
                canonical[col_idx] = to_u
                converted = True
                break
        if not converted:
            raise ValueError(
                f"Unknown / ambiguous unit '{unit}' in column {col_idx} of {path}"
            )

    meta['units'] = ', '.join(canonical)

    time = raw[:, 0]
    data_cols = raw[:, 1:] if raw.shape[1] > 1 else raw[:, 0:0]
    return time, data_cols, meta


def write_manifest(path, *, model, description, sources, normalized_datasets,
                   simulation_parameters=None):
    """Write a manifest.json for a verification dataset.

    Args:
        path:                   Output file (manifest.json).
        model:                  Verification case name.
        description:            Human-readable description.
        sources:                List of dicts with tool/version/raw_files/notes.
        normalized_datasets:    List of dicts with file/original_source/etc.
        simulation_parameters:  Optional dict of simulation settings.
    """
    payload = {
        "schema_version": SCHEMA_VERSION,
        "model": model,
        "description": description,
        "sources": sources,
        "normalized_datasets": normalized_datasets,
    }
    if simulation_parameters:
        payload["simulation_parameters"] = simulation_parameters

    Path(path).parent.mkdir(parents=True, exist_ok=True)
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(payload, f, indent=2)
        f.write('\n')


def read_manifest(path):
    """Read and return the contents of a manifest.json."""
    with open(path, 'r', encoding='utf-8') as f:
        return json.load(f)
