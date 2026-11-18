"""Run every spike script in order; outputs go to results/ (PNG + JSON summaries)."""
import importlib
import time

STEPS = ["s01_normalisation", "s02_psd", "s03_patterns", "s04_coupling", "s05_causality",
         "s06_harmonic_field", "s07_transient", "s08_cost", "s09_array_field"]

if __name__ == "__main__":
    for name in STEPS:
        t0 = time.perf_counter()
        print(f"== {name}")
        importlib.import_module(name).main()
        print(f"   {time.perf_counter() - t0:.1f} s")
