Done. Here's what the reframe adds and what bugs were found:

**Reframing applied:**
- The file header derives both transfer functions H_α(s) and H_β(s) from first principles, showing they form a complementary all-pass pair projecting onto {sin(ωt), cos(ωt)}
- `updateCoefficients` is annotated with the full Tustin derivation — each coefficient mapped to its position in D(z)
- `FrequencyAdaptivePLL::update` names the discriminator as a normalised cross-product with its small-angle linearisation explicitly stated
- `AdaptivePLL::update` names the gain adaptation algorithm precisely: it's NLMS with a 1-sample ARX model

**Bugs found:**
1. **`integral_state = i_term / ki` with ki=0** — divide-by-zero in the anti-windup back-calculation path. Both clamp branches have it
2. **`p_scale` and `learn_scale` uninitialised** — set externally by `setDistortionDamping()` but have no safe default in `AdaptivePLL::init()`, making first-call behaviour undefined
3. **No frequency clamp in `AdaptivePLL::update`** — unlike the base class, the adaptive loop can push `freq` outside the ±50% band under large `gain_est`

**Structural shortcomings flagged:**
- Dual independent state registers (`wz1_a/wz2_a` and `wz1_b/wz2_b`) share identical denominators — one shared state variable would suffice
- `coeff_valid` is never cleared when ω changes — filter runs stale coefficients silently
- The history ring `SOGI_HIST_LEN` is over-provisioned; only one step back is ever read
