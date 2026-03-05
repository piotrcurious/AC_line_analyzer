import subprocess
import numpy as np
import matplotlib.pyplot as plt
import csv
import io

def run_simulation(v_signal, i_signal):
    # Prepare input data for the harness
    input_str = ""
    for v, i in zip(v_signal, i_signal):
        input_str += f"{v} {i}\n"

    # Run the harness
    process = subprocess.Popen(
        ['./sogi/modular_dual_k6b_resample_optimized6_no2_opt2_phase4/tests/harness'],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True
    )
    stdout_data, stderr_data = process.communicate(input=input_str)

    if process.returncode != 0:
        print(f"Harness failed with return code {process.returncode}")
        print(stderr_data)
        return None

    # Parse output
    results = []
    cycle_ends = []
    for line in stdout_data.splitlines():
        parts = line.split(',')
        if parts[0] == 'V_SAMPLE':
            results.append({
                'v_val': float(parts[1]),
                'freq': float(parts[2]),
                'mag1': float(parts[3]),
                'ratio': float(parts[4]),
                'gain_est': float(parts[5])
            })
        elif parts[0] == 'CYCLE_END':
            cycle_ends.append({
                'freq': float(parts[1]),
                'v_dc': float(parts[2]),
                'spc': int(parts[3])
            })

    return results, cycle_ends

def test_freq_step():
    fs = 125000 # 250k total / 2 channels
    duration = 1.0 # seconds
    t = np.arange(0, duration, 1/fs)

    # Frequency step: 50Hz -> 52Hz at 0.5s
    freq = np.where(t < 0.5, 50.0, 52.0)
    phase = 2 * np.pi * np.cumsum(freq) / fs

    # 2000mV centered sine wave (1650mV DC + 350mV amplitude)
    v_signal = 1650 + 350 * np.sin(phase)
    i_signal = 1650 + 200 * np.sin(phase - np.pi/4)

    results, cycle_ends = run_simulation(v_signal, i_signal)

    if results:
        freqs = np.array([r['freq'] for r in results])

        # Programmatic verification
        final_freq = np.mean(freqs[-100:])
        error = abs(final_freq - 52.0)
        print(f"Freq Step Test: Final Freq = {final_freq:.4f} Hz, Error = {error:.4f} Hz")

        # Convergence time (index where it reaches 51.9Hz)
        conv_idx = np.where(freqs > 51.9)[0]
        if len(conv_idx) > 0:
            print(f"Freq Step Test: Convergence at index {conv_idx[0]}")

        plt.figure(figsize=(10, 6))
        plt.plot(freqs, label='Tracked Frequency')
        plt.axhline(50, color='r', linestyle='--', label='Initial Freq')
        plt.axhline(52, color='g', linestyle='--', label='Target Freq')
        plt.title('Frequency Step Response (50Hz -> 52Hz)')
        plt.xlabel('Virtual Sample Index')
        plt.ylabel('Frequency (Hz)')
        plt.legend()
        plt.grid(True)
        plt.savefig('sogi/modular_dual_k6b_resample_optimized6_no2_opt2_phase4/tests/freq_step.png')
        print("Frequency step test completed. Plot saved to tests/freq_step.png")

def test_harmonics():
    fs = 125000
    duration = 1.0
    t = np.arange(0, duration, 1/fs)

    # 50Hz fundamental + 30% 3rd harmonic added at 0.5s
    v_fund = 350 * np.sin(2 * np.pi * 50 * t)
    v_harm = np.where(t < 0.5, 0, 105 * np.sin(2 * np.pi * 150 * t))
    v_signal = 1650 + v_fund + v_harm
    i_signal = 1650 + 200 * np.sin(2 * np.pi * 50 * t)

    results, cycle_ends = run_simulation(v_signal, i_signal)

    if results:
        ratios = np.array([r['ratio'] for r in results])
        freqs = np.array([r['freq'] for r in results])

        # Programmatic verification
        max_ratio = np.max(ratios)
        steady_ratio = np.mean(ratios[-100:])
        print(f"Harmonics Test: Max H3 Ratio = {max_ratio:.4f}, Steady State Ratio = {steady_ratio:.4f}")

        fig, ax1 = plt.subplots(figsize=(10, 6))

        ax1.set_xlabel('Virtual Sample Index')
        ax1.set_ylabel('Tracked Frequency (Hz)', color='tab:blue')
        ax1.plot(freqs, color='tab:blue', label='Frequency')
        ax1.tick_params(axis='y', labelcolor='tab:blue')

        ax2 = ax1.twinx()
        ax2.set_ylabel('H3 Ratio', color='tab:red')
        ax2.plot(ratios, color='tab:red', label='H3 Ratio')
        ax2.tick_params(axis='y', labelcolor='tab:red')

        plt.title('Harmonic Distortion Test (30% H3 at 0.5s)')
        plt.grid(True)
        plt.savefig('sogi/modular_dual_k6b_resample_optimized6_no2_opt2_phase4/tests/harmonics.png')
        print("Harmonics test completed. Plot saved to tests/harmonics.png")

if __name__ == "__main__":
    test_freq_step()
    test_harmonics()
