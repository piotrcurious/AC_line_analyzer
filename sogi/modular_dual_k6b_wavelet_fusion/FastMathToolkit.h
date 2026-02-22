/* * FastMathToolkit.h (Ring / Formal-series extension)
 *
 * Single-file ESP32/Arduino-compatible header implementing:
 * - Ring tables: float scalars embedded into the Q16.16 ring
 * as precomputed multipliers (endomorphisms x ↦ c·x).
 * - Variable float multiplication using the ring-table framework:
 *
 * f = sign · 2^e · m, where m ∈ [1,2)
 *
 * The mantissa m is approximated by a piecewise ring chart table
 * generated offline. Each segment stores:
 * - a Q16.16 multiplier for the mantissa center
 * - an optional linear correction slope (Q16.16)
 *
 * Usage: uint32_t r = mul_u32_by_float_ring(a, f);
 */

#ifndef FAST_RING_TABLES_H
#define FAST_RING_TABLES_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <math.h>
#include <limits.h>

// ------------------------------------------------------------------
// Configuration
// ------------------------------------------------------------------
#ifndef RING_MANT_SEGMENTS
#define RING_MANT_SEGMENTS 16
#endif

// Mantissa multipliers are stored as Q16.16
#define RING_Q 16
#define RING_SCALE (1u << RING_Q)

#ifdef __cplusplus
extern "C" {
#endif

// ------------------------------------------------------------------
// Ring mantissa table (generated offline)
// ------------------------------------------------------------------
// Each segment stores:
//   c0 = mantissa center multiplier (Q16.16)
//   c1 = linear slope correction     (Q16.16)
//
// Approximation:
//   m ≈ c0 + c1 * δ
// where δ is the local offset inside the segment in Q16.
//
// This is a first-order formal-series chart per interval.

typedef struct {
    uint32_t c0_q16;
    int32_t  c1_q16;
} RingMantSeg;

static const RingMantSeg RING_MANT_TABLE[RING_MANT_SEGMENTS] = {
    {0x00010800u, 0x00001000}, {0x00011800u, 0x00001000},
    {0x00012800u, 0x00001000}, {0x00013800u, 0x00001000},
    {0x00014800u, 0x00001000}, {0x00015800u, 0x00001000},
    {0x00016800u, 0x00001000}, {0x00017800u, 0x00001000},
    {0x00018800u, 0x00001000}, {0x00019800u, 0x00001000},
    {0x0001A800u, 0x00001000}, {0x0001B800u, 0x00001000},
    {0x0001C800u, 0x00001000}, {0x0001D800u, 0x00001000},
    {0x0001E800u, 0x00001000}, {0x0001F800u, 0x00001000}
};

// ------------------------------------------------------------------
// Low-level ring multiply
// ------------------------------------------------------------------
static inline uint32_t mul_u32_by_q16(uint32_t a, uint32_t mul_q16) {
    uint64_t prod = (uint64_t)a * (uint64_t)mul_q16;
    return (uint32_t)(prod >> 16);
}

// ------------------------------------------------------------------
// Core: multiply uint32 by variable float using ring-table framework
// ------------------------------------------------------------------
static inline uint32_t mul_u32_by_float_ring(uint32_t a, float f) {
    if (a == 0u) return 0u;
    if (!isfinite(f) || f == 0.0f) return 0u;

    // ---- Step 1: extract sign (unsigned saturating logic) ----
    if (f < 0.0f) return 0u;

    // ---- Step 2: bit-level float decomposition ----
    union {
        float f;
        uint32_t u;
    } x;
    x.f = f;

    int exp = (int)((x.u >> 23) & 0xFF) - 127;
    uint32_t mant = (x.u & 0x7FFFFFu) | 0x800000u; // implicit 1

    // mantissa in Q1.23 representing [1,2)
    // Get segment index from top bits (assuming 16 segments = 4 bits)
    int idx = (mant >> (23 - 4)) & (RING_MANT_SEGMENTS - 1);

    // local offset δ inside segment (Q16)
    uint32_t frac_mask = (1u << (23 - 4)) - 1;
    uint32_t frac = mant & frac_mask;
    uint32_t delta_q16 = frac >> ((23 - 4) - 16);

    // ---- Step 3: evaluate local chart m ≈ c0 + c1·δ ----
    RingMantSeg seg = RING_MANT_TABLE[idx];

    int64_t corr = (int64_t)seg.c1_q16 * (int64_t)delta_q16;
    uint32_t m_q16 = seg.c0_q16 + (uint32_t)(corr >> 16);

    // ---- Step 4: multiply integer by mantissa multiplier ----
    uint32_t r = mul_u32_by_q16(a, m_q16);

    // ---- Step 5: apply exponent via shift ----
    if (exp > 0) {
        if (exp >= 32) return UINT32_MAX;
        uint64_t up = (uint64_t)r << exp;
        return (up > 0xFFFFFFFFull) ? UINT32_MAX : (uint32_t)up;
    } else if (exp < 0) {
        int sh = -exp;
        if (sh >= 32) return 0u;
        return r >> sh;
    }

    return r;
}

static inline uint32_t mul_u32_by_float(uint32_t a, float f) {
    return mul_u32_by_float_ring(a, f);
}

#ifdef __cplusplus
}
#endif

#endif // FAST_RING_TABLES_H
