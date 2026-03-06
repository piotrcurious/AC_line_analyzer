Now I have the full picture. Let me rewrite it with the unified transform framework framing — everything expressed as operators, projections, and compositions, with bugs clearly annotated.

The rewrite is done — no functional changes, only the conceptual framing. Here's a summary of what changed and what bugs are flagged:

**Reframing applied:**
- Every subsystem is now named as the operator it actually is: `T_adc`, `T_resamp`, `P_sogi`, `P_sogi3`, `Φ_pll`
- SOGI is described as a bandpass projection onto the quadrature subspace `{sin(ωt), cos(ωt)}` — the Haar-like inner product framing you described
- Phase wrap is explicitly described as lifting from S¹ to its covering space ℝ
- The `atan2` sign convention on `v_beta` is explained from first principles
- The distortion gate is named as a gain-scheduled nonlinearity with a describing-function approximation

**Bugs flagged inline:**
1. `HARMONIC_SMOOTH_ALPHA = 0.99` — almost certainly inverted; this gives near-zero smoothing
2. `PLL_KI = 0` — Type-0 loop, steady-state frequency error is guaranteed
3. `logTask` has an unsynchronised cross-core read of the harmonic globals — true data race on dual-core ESP32
4. `phase_offset` grows unbounded — float32 precision degrades after ~30 hours
5. Zero-order hold path (`acc_count == 0`) leaves stale data in `v_buf`/`i_buf` rather than filling with neutral DC
6. `learn_att` is binary — step discontinuity in loop gain at threshold
