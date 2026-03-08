import numpy as np
import subprocess
import matplotlib.pyplot as plt
import os

def generate_signal(duration=1.0, fs=250000, freq=50.0, wave_type='sine'):
    t = np.arange(0, duration, 1/fs)
    if wave_type == 'sine':
        v = 1650 + 1000 * np.sin(2 * np.pi * freq * t)
    elif wave_type == 'square':
        v = 1650 + 1000 * np.sign(np.sin(2 * np.pi * freq * t))
    elif wave_type == 'trapezoid':
        # Trapezoid is a clipped sine or integrated square.
        # Let's use a clipped sine for a trapezoid-like appearance.
        v_raw = 1500 * np.sin(2 * np.pi * freq * t)
        v = 1650 + np.clip(v_raw, -1000, 1000)
    elif wave_type == 'complex_transition':
        # Sine -> Square -> Trapezoid -> Sine (at different freq)
        t1 = t[:len(t)//4]
        t2 = t[len(t)//4:2*len(t)//4]
        t3 = t[2*len(t)//4:3*len(t)//4]
        t4 = t[3*len(t)//4:]

        v1 = 1650 + 1000 * np.sin(2 * np.pi * freq * t1)
        v2 = 1650 + 1000 * np.sign(np.sin(2 * np.pi * freq * t2))
        v3 = 1650 + np.clip(1500 * np.sin(2 * np.pi * freq * t3), -1000, 1000)
        v4 = 1650 + 1000 * np.sin(2 * np.pi * (freq+10.0) * t4)
        v = np.concatenate([v1, v2, v3, v4])
    return v

def run_sim(v_signal, tool_path):
    input_str = ""
    for val in v_signal:
        input_str += f"{val} {1650}\n"

    process = subprocess.Popen([tool_path], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    stdout, stderr = process.communicate(input=input_str)

    if stderr:
        print("Error:", stderr)

    data_list = []
    cycle_list = []
    for line in stdout.splitlines():
        parts = line.split(',')
        if parts[0] == 'DATA':
            data_list.append([float(x) for x in parts[1:]])
        elif parts[0] == 'CYCLE':
            cycle_list.append([float(x) for x in parts[1:]])

    data_arr = np.array(data_list)
    cycle_arr = np.array(cycle_list)
    return data_arr, cycle_arr

def main():
    # Use environment variable or default to local path
    tool_path = os.environ.get('HARNESS_TOOL', './harness_tool')
    fs = 250000
    duration = 2.0 # Longer duration for complex transition

    print("Testing complex transition (Sine -> Square -> Trapezoid -> Sine @ 60Hz)...")
    v = generate_signal(duration=duration, fs=fs, wave_type='complex_transition', freq=50.0)
    data, cycle = run_sim(v, tool_path)

    # columns: 0:v_val, 1:freq, 2:v_alpha, 3:u1_dc_removed, 4:gain_est, 5:v_dc, 6:samples_per_cycle
    freq = data[:, 1]
    v_val = data[:, 0]
    u1_fundamental = data[:, 2]
    gain_est = data[:, 4]
    spc = data[:, 6]

    plt.figure(figsize=(12, 12))

    plt.subplot(4, 1, 1)
    plt.plot(freq, label='Tracked Frequency (Hz)')
    plt.axhline(50, color='r', linestyle='--', alpha=0.5, label='Target 50Hz')
    plt.axhline(60, color='g', linestyle='--', alpha=0.5, label='Target 60Hz')
    plt.title('SOGI-FLL Frequency Tracking Performance (Phase 6 UTF)')
    plt.ylabel('Frequency (Hz)')
    plt.legend()
    plt.grid(True)

    plt.subplot(4, 1, 2)
    plt.plot(spc, label='Samples Per Cycle (N)')
    plt.title('Adaptive Virtual Grid (T_resamp)')
    plt.ylabel('N')
    plt.legend()
    plt.grid(True)

    plt.subplot(4, 1, 3)
    plt.plot(v_val, label='Input Signal (v_val)', alpha=0.5)
    plt.plot(u1_fundamental, label='Fundamental (v_alpha)', linewidth=2)
    plt.title('Signal and Fundamental Component (Total Run)')
    plt.ylabel('mV')
    plt.legend()
    plt.grid(True)

    plt.subplot(4, 1, 4)
    plt.plot(gain_est, label='Online Gain Estimate')
    plt.title('NLMS Sensitivity Identification')
    plt.ylabel('Gain')
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.savefig('phase6_verification.png')
    print("Verification plot saved to phase6_verification.png")

if __name__ == "__main__":
    main()
