import numpy as np

def sogi_coeffs(omega, ts, k=0.7071):
    wts = omega * ts
    wts2 = wts * wts
    k_wts = k * wts
    det = 1.0 / (4.0 + 2.0 * k_wts + wts2)

    a_b0 = 2.0 * k_wts * det
    a_a1 = 2.0 * (wts2 - 4.0) * det
    a_a2 = (4.0 - 2.0 * k_wts + wts2) * det

    b_b0 = (k * wts2) * det
    b_b1 = 2.0 * b_b0

    return a_b0, a_a1, a_a2, b_b0, b_b1

def sogi_response(f_sig, f_res, fs, k=0.7071):
    ts = 1.0/fs
    omega = 2 * np.pi * f_res
    a_b0, a_a1, a_a2, b_b0, b_b1 = sogi_coeffs(omega, ts, k)

    z = np.exp(1j * 2 * np.pi * f_sig * ts)

    # H_alpha(z) = a_b0 * (1 - z^-2) / (1 + a_a1*z^-1 + a_a2*z^-2)
    h_alpha = a_b0 * (1 - z**-2) / (1 + a_a1 * z**-1 + a_a2 * z**-2)

    # H_beta(z) = (b_b0 + b_b1*z^-1 + b_b0*z^-2) / (1 + a_a1*z^-1 + a_a2*z^-2)
    h_beta = (b_b0 + b_b1 * z**-1 + b_b0 * z**-2) / (1 + a_a1 * z**-1 + a_a2 * z**-2)

    return h_alpha, h_beta

fs = 6400
f_res = 50
f_sig = 50

ha, hb = sogi_response(f_sig, f_res, fs)
print(f"At {f_sig}Hz (Resonant {f_res}Hz, Fs {fs}Hz):")
print(f"  Alpha: Gain {abs(ha):.4f}, Phase {np.angle(ha, deg=True):.2f} deg")
print(f"  Beta:  Gain {abs(hb):.4f}, Phase {np.angle(hb, deg=True):.2f} deg")

ha_hi, hb_hi = sogi_response(51, 50, fs)
print(f"At 51Hz (Resonant 50Hz):")
print(f"  Alpha: Gain {abs(ha_hi):.4f}, Phase {np.angle(ha_hi, deg=True):.2f} deg")
print(f"  Beta:  Gain {abs(hb_hi):.4f}, Phase {np.angle(hb_hi, deg=True):.2f} deg")

# Discriminator check: (v_alpha - u) * v_beta
# u = sin(wt) = 1.0 @ 90 deg? No, let u = 1.
# v_alpha = ha * u, v_beta = hb * u
# Error = Real( (ha - 1) * conj(hb) ) / 2  ?
# No, time domain: u = cos(wt), v_alpha = |ha| cos(wt + angle(ha)), v_beta = |hb| cos(wt + angle(hb))
# Error = avg( (v_alpha - u) * v_beta )
t = np.linspace(0, 1/50, 1000)
u = np.cos(2*np.pi*51*t)
va = abs(ha_hi) * np.cos(2*np.pi*51*t + np.angle(ha_hi))
vb = abs(hb_hi) * np.cos(2*np.pi*51*t + np.angle(hb_hi))
err = np.mean((va - u) * vb)
print(f"Discriminator (v_alpha - u) * v_beta at 51Hz: {err:.6f}")

u_lo = np.cos(2*np.pi*49*t)
ha_lo, hb_lo = sogi_response(49, 50, fs)
va_lo = abs(ha_lo) * np.cos(2*np.pi*49*t + np.angle(ha_lo))
vb_lo = abs(hb_lo) * np.cos(2*np.pi*49*t + np.angle(hb_lo))
err_lo = np.mean((va_lo - u_lo) * vb_lo)
print(f"Discriminator (v_alpha - u) * v_beta at 49Hz: {err_lo:.6f}")
