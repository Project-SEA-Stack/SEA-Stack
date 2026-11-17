"""List the datasets relevant to radiated-wave visualisation in each demo .h5.

Usage: python inventory.py [root]
"""
import pathlib
import sys

import h5py

ROOT = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else
                    pathlib.Path(__file__).resolve().parents[3] / "data" / "demos" / "run_seastack")


def describe(path):
    with h5py.File(path, "r") as f:
        print(f"\n== {path.relative_to(ROOT)}")
        for k in ("simulation_parameters",):
            if k in f:
                for n, d in f[k].items():
                    if isinstance(d, h5py.Dataset) and d.size <= 50:
                        print(f"   {k}/{n} = {d[()]}  {dict((a, d.attrs[a]) for a in d.attrs)}")
        b1 = f.get("body1")
        if b1 is None:
            return
        def visit(name, obj):
            if isinstance(obj, h5py.Dataset):
                attrs = {a: (obj.attrs[a] if len(str(obj.attrs[a])) < 80 else "...") for a in obj.attrs}
                print(f"   body1/{name} {obj.shape} {attrs}")
        b1.visititems(visit)


for p in sorted(ROOT.glob("*/assets/hydroData/*.h5")):
    describe(p)
