# ETA import test notes (SEA-Stack)

Deep dive on the DFT eta-import pathway and how it relates to verification and comparison tests. See also the `sphere_irreg_waves_eta` row in [`../../TEST_SUITES_REFERENCE.md`](../../TEST_SUITES_REFERENCE.md).

The `sphere_irreg_waves_eta` regression test exercises the DFT eta-import pathway
(`BuildFromEtaFile`) with relaxed tolerances (L2 ~ 1.7e-3, Linf ~ 1.2). This elevated
error is a **known limitation** of the DFT reconstruction, not a bug.

**Root cause:**

- The targeted DFT uses nf=1000 frequencies from 0.001--1.0 Hz
- Spectral leakage occurs when the DFT grid doesn't align with original components
- Broadband irregular waves are most affected; narrow-band spectra have lower error

**Diagnostics:**

- `tests/regression/sphere/diagnose_eta_failure.py` — detailed analysis and hypotheses
- Status files include a `"note"` field documenting this limitation

## Verification vs comparison strategy for eta-import

- The **RM3 MoorDyn verification** test uses the direct convolution pathway
  (`EtaTableWaveField`) — no DFT reconstruction. This ensures reliable cross-code
  comparison against WEC-Sim/MoorDyn without spectral approximation artifacts.
- The **DFT vs convolution comparison** test (`compare_eta_dft_vs_convolution_sphere_irreg`)
  quantifies the reconstruction error of the DFT pathway independently, with non-gating
  acceptance criteria.
- The DFT pathway remains intact in `ComponentSampler::BuildFromEtaFile` and is exercised by
  the `sphere_irreg_waves_eta` regression test.

## Excitation pathway summary

| Pathway | Wave model | Excitation | When to use |
|---------|-----------|------------|-------------|
| Direct eta convolution | `EtaTableWaveField` | IRF slow path | Verification with imported eta (faithful reproduction) |
| DFT decomposition | `BuildFromEtaFile` + `LinearDirectionalWaveField` | IRF fast path | Spectral analysis of imported eta (introduces reconstruction error) |
| Spectrum-based | `ComponentSampler::Build` + `LinearDirectionalWaveField` | IRF fast path | Standard parametric sea states (JONSWAP, PM, etc.) |
