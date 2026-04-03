#!/usr/bin/env python3
"""
Sphere Regular Waves Regression Test Comparison Script

This script compares the results of the sphere regular waves test against reference data.
The test runs 10 different wave conditions, so we need to compare each one.
"""

import sys
import os
import glob
from pathlib import Path

# Add the utilities directory to the path to import the comparison template
sys.path.append(str(Path(__file__).parent.parent / "utilities"))
from compare_template import run_comparison, run_multi_column_comparison

def main():
    """Main comparison function for sphere regular waves test."""
    
    # Parse arguments: support both old format (ref_file, test_file) and new format with --condition
    requested_condition = None
    if len(sys.argv) == 5 and sys.argv[3] == "--condition":
        ref_file = sys.argv[1]
        results_file = sys.argv[2]
        requested_condition = int(sys.argv[4])
    elif len(sys.argv) == 3:
        ref_file = sys.argv[1]
        results_file = sys.argv[2]
    else:
        print("Usage: python compare_sphere_reg_waves.py <reference_file> <test_file> [--condition N]")
        sys.exit(1)

    print("Reference file: ", ref_file)
    print("Results file:   ", results_file)
    if requested_condition:
        print(f"Comparing condition: {requested_condition}")

    # Get the reference data directory from the reference file
    ht = os.path.split(ref_file)
    ref_dir = Path(ht[0])
    
    if not ref_dir.exists():
        print(f"Error: Reference directory not found: {ref_dir}")
        sys.exit(1)
    
    # Get the results directory from the results file
    ht = os.path.split(results_file)
    results_dir = Path(ht[0])
    
    if not results_dir.exists():
        print(f"Error: Results directory not found: {results_dir}")
        sys.exit(1)
    
    # Find result files produced by the C++ test
    # C++ writes files named: results_sphere_reg_waves_<N>.txt
    if requested_condition:
        # Single condition mode: only compare the requested condition
        result_files = [results_dir / f"results_sphere_reg_waves_{requested_condition}.txt"]
        if not result_files[0].exists():
            print(f"Error: Result file not found: {result_files[0]}")
            sys.exit(1)
    else:
        # Multi-condition mode: find all result files
        result_files = list(results_dir.glob("results_sphere_reg_waves_*.txt"))
        result_files.sort()
        
        if not result_files:
            print(f"Error: No result files found in {results_dir}")
            sys.exit(1)
    
    print(f"Found {len(result_files)} result file(s) in {results_dir}")
    
    # Compare each wave condition
    all_passed = True
    
    for result_file in result_files:
        # Extract wave number from filename
        wave_num = result_file.stem.split('_')[-1]
        ref_file = ref_dir / f"ss_ref_sphere_reg_waves_{wave_num}.txt"
        
        if not ref_file.exists():
            print(f"Warning: Reference file {ref_file} not found, skipping wave {wave_num}")
            continue
        
        print(f"\nComparing wave condition {wave_num}...")
        print(f"  Reference: {ref_file}")
        print(f"  Result:    {result_file}")
        
        # Run comparison using the template
        try:
            n1, n2, passed = run_comparison(
                str(ref_file),
                str(result_file),
                test_name=f"Sphere Regular Waves - Wave {wave_num}",
                y_label="Heave (m)",
                executable_patterns=["sphere_reg_waves_test"],
                pass_criteria=(1e-4, 0.02),
                status_name=f"sphere_reg_waves_wave_{wave_num}"
            )
            
            if not passed:
                all_passed = False
                print(f"  FAILED Wave {wave_num} comparison")
            else:
                print(f"  PASSED Wave {wave_num} comparison")
                
        except Exception as e:
            print(f"  ERROR comparing wave {wave_num}: {e}")
            all_passed = False
    
    if all_passed:
        print("\nAll wave conditions passed comparison!")
        sys.exit(0)
    else:
        print("\nSome wave conditions failed comparison!")
        sys.exit(1)

if __name__ == "__main__":
    main() 