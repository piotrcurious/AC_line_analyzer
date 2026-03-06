import numpy as np
import matplotlib.pyplot as plt
import subprocess
import csv
import os

# --- Simulation Parameters ---
fs_adc = 125000  # 125kHz per channel
duration = 1.0   # seconds
t = np.arange(0, duration, 1/fs_adc)

def run_sim(signal):
    with open("input.txt", "w") as f:
        for v in signal:
            f.write(f"{v + 1650.0} 1650.0\n") # Center at 1650mV

    # Compile if needed
    subprocess.run(["g++", "-O3", "-o", "harness", "harness.cpp", "../SOGI.cpp"], check=True)

    # Run harness
    with open("input.txt", "r") as fin, open("sim_output.csv", "w") as fout:
        subprocess.run(["./harness"], stdin=fin, stdout=fout, check=True)

    results = []
    with open("sim_output.csv", "r") as f:
        reader = csv.reader(f)
        for row in reader:
            if row[0] == "V_SAMPLE":
                results.append([float(x) for x in row[1:]])
    return np.array(results)

def test_distortion_event():
    print("Testing Rapid Waveform Change (Sine -> Square)...")

    # Signal: 0.5s Sine (50Hz), then 0.5s Square (50Hz)
    f_sig = 50.0
    amp = 1000.0 # 1V amplitude

    sine_part = amp * np.sin(2 * np.pi * f_sig * t[:len(t)//2])
    square_part = amp * np.sign(np.sin(2 * np.pi * f_sig * t[len(t)//2:]))
    signal = np.concatenate([sine_part, square_part])

    res = run_sim(signal)

    plt.figure(figsize=(10, 6))
    plt.plot(res[:, 1], label="Tracked Frequency (Hz)")
    plt.axhline(50.0, color='r', linestyle='--', label="Actual Freq (50Hz)")
    plt.title("Frequency Stability during Rapid Waveform Change (Sine -> Square)")
    plt.xlabel("Virtual Sample Index")
    plt.ylabel("Frequency (Hz)")
    plt.grid(True)
    plt.legend()
    plt.savefig("distortion_event.png")
    plt.close()

    # Error metrics
    final_freq = res[-1, 1]
    error = abs(final_freq - 50.0)
    print(f"Final Frequency: {final_freq:.4f} Hz (Error: {error:.4f} Hz)")

    # Check if frequency stayed within reasonable bounds (e.g., 49-51 Hz)
    max_f = np.max(res[len(res)//2:, 1])
    min_f = np.min(res[len(res)//2:, 1])
    print(f"Max Frequency in Square wave: {max_f:.4f} Hz")
    print(f"Min Frequency in Square wave: {min_f:.4f} Hz")

if __name__ == "__main__":
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    test_distortion_event()
