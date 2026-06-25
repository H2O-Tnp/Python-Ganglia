"""
model_fitter.py — Nonlinear Motor Model Fitting
================================================
Loads all sysid_data/*.ndjson telemetry files, fits a 25-parameter
electro-mechanical motor model using a staged SciPy cascade, then writes:

  - motor_model.json   — machine-readable model parameters
  - sysid_report/      — diagnostic PNG plots

Physics model fitted:
  di/dt = (u - R·i - Ke·ω) / L
  dω/dt = (Kt·i - Tf(ω,θ)) / J
  dθ/dt = ω

  Tf(ω,θ) = Fc·sign(ω)·(1-exp(-|ω|/ωs))   ← Coulomb
           + Fs·exp(-(ω/ωs)²)·sign(ω)        ← Stribeck peak
           + B·ω                               ← Viscous
           + Σᵢ Aᵢ·sin(Nᵢ·θ+φᵢ)             ← Cogging harmonics

Noise model:  σ(ω) = σ0 + σ1·|ω|

Usage:
    .venv/bin/python model_fitter.py [--data-dir sysid_data] [--no-plots]
"""
import argparse
import json
import math
import pathlib
import sys
import numpy as np
from scipy.optimize import curve_fit, differential_evolution
from scipy.signal import find_peaks
from scipy.fft import rfft, rfftfreq

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    HAS_MPL = True
except ImportError:
    HAS_MPL = False
    print("Note: matplotlib not found — plots will be skipped.")

DATA_DIR   = pathlib.Path("sysid_data")
REPORT_DIR = pathlib.Path("sysid_report")
REPORT_DIR.mkdir(exist_ok=True)

# ── physical constants used across stages ─────────────────────────────────────
VEL_SCALE = 10.0          # firmware fixed-point scale
ADC_TO_MA = 4.698555425   # same as main.py
PWM_FULL  = 4000.0        # firmware full-scale PWM value
MAX_VOLTAGE = 24.0        # supply voltage


# ──────────────────────────────────────────────────────────────────────────────
# Data loading
# ──────────────────────────────────────────────────────────────────────────────
def load_data(data_dir: pathlib.Path) -> dict[str, list[dict]]:
    """Load all NDJSON files, keyed by test-name prefix."""
    files = sorted(data_dir.glob("*.ndjson"))
    if not files:
        print(f"ERROR: No .ndjson files in {data_dir}")
        sys.exit(1)
    datasets: dict[str, list[dict]] = {}
    for f in files:
        key = f.stem.split("_")[0] + "_" + f.stem.split("_")[1]  # e.g. "01_prbs"
        recs = []
        with open(f) as fp:
            for line in fp:
                line = line.strip()
                if line:
                    try:
                        recs.append(json.loads(line))
                    except json.JSONDecodeError:
                        pass
        datasets[key] = recs
        print(f"  Loaded {len(recs):6d} pts ← {f.name}")
    return datasets


def to_arrays(recs: list[dict]) -> tuple[np.ndarray, ...]:
    """Convert record list to numpy arrays: t, vel_rpm, cur_ma, target_rpm."""
    t   = np.array([r.get("timestamp", r.get("time", 0.0)) for r in recs])
    vel = np.array([r.get("velocity", 0.0) for r in recs])  # already in RPM
    cur = np.array([r.get("current", 0.0) for r in recs])   # already in mA
    agent_tgts = np.array([r.get("agent_target", 0.0) for r in recs])
    if np.std(agent_tgts) > 0.1:
        tgt = agent_tgts
    else:
        tgt = np.array([r.get("target_velocity", 0.0) for r in recs])
    t = t - t[0]
    return t, vel, cur, tgt


# ──────────────────────────────────────────────────────────────────────────────
# Friction torque model
# ──────────────────────────────────────────────────────────────────────────────
def friction_torque(omega_rpm: np.ndarray, theta_rad: np.ndarray,
                    Fc, Fs, omega_s_rpm, B,
                    A1, A2, A3, A4, A5, A6,
                    N1, N2, N3, N4, N5, N6,
                    phi1, phi2, phi3, phi4, phi5, phi6) -> np.ndarray:
    """Full nonlinear friction + cogging torque (Nm-equivalent in RPM units)."""
    # Coulomb + Stribeck
    omega_r = np.where(np.abs(omega_rpm) < 1e-6, 1e-6, omega_rpm)
    Tf = Fc * np.sign(omega_rpm) * (1.0 - np.exp(-np.abs(omega_rpm) / (omega_s_rpm + 1e-9)))
    Tf += Fs * np.exp(-(omega_rpm / (omega_s_rpm + 1e-9))**2) * np.sign(omega_rpm)
    # Viscous
    Tf += B * omega_rpm
    # Cogging harmonics
    Ns = [N1, N2, N3, N4, N5, N6]
    As = [A1, A2, A3, A4, A5, A6]
    phis = [phi1, phi2, phi3, phi4, phi5, phi6]
    for A, N, phi in zip(As, Ns, phis):
        Tf += A * np.sin(N * theta_rad + phi)
    return Tf


# ──────────────────────────────────────────────────────────────────────────────
# Stage 1 — Electrical constants (R, L, Ke)
# ──────────────────────────────────────────────────────────────────────────────
def fit_electrical(datasets: dict) -> dict:
    print("\n── Stage 1: Electrical constants (R, L, Ke) ──")
    # Use electrical step data if available
    key = next((k for k in datasets if "08" in k or "electrical" in k), None)
    if key is None:
        print("  No electrical_step data — using defaults.")
        return {"R": 4.8, "L": 0.002, "Ke": 0.005}

    t, vel, cur, tgt = to_arrays(datasets[key])
    
    # Estimate R from the 95th percentile of positive currents under low-speed
    # during active 12V bursts (cur > 10.0 mA, abs(vel) < 5.0)
    low_speed_mask = np.abs(vel) < 5.0
    active_cur = cur[low_speed_mask & (cur > 10.0)]
    
    if len(active_cur) > 10:
        I_ss = np.percentile(active_cur, 95) / 1000.0  # Convert mA to A
        V_approx = 2000.0 / PWM_FULL * MAX_VOLTAGE  # 12 V
        R_est = V_approx / I_ss if I_ss > 0.01 else 4.8
    else:
        R_est = 4.8

    # Use a fixed physically representative default for inductance L (2 mH)
    L_est = 0.002

    Ke_est = 0.005  # fallback — will be refined in free-decel stage
    # Refine Ke from free-decel if available
    decel_key = next((k for k in datasets if "09" in k or "decel" in k), None)
    if decel_key:
        t2, vel2, cur2, _ = to_arrays(datasets[decel_key])
        # Under short-circuit braking (PWM=0), V_term ≈ 0.
        # Physics: V_term = R*I_A + L*dI_A/dt + Ke*omega ≈ 0
        # => Ke*omega ≈ -R*I_A. We fit Ke from slope of -R*I_A vs omega (rad/s)
        omega = vel2 * 2 * math.pi / 60.0  # rad/s
        I_A = cur2 / 1000.0
        y_fit = -R_est * I_A
        try:
            # Fit Ke as the slope of y_fit vs omega
            slope, _ = np.polyfit(omega, y_fit, 1)
            if 0.001 < slope < 1.0:
                Ke_est = slope
        except Exception:
            pass

    result = {"R": round(R_est, 4), "L": round(L_est, 6), "Ke": round(Ke_est, 6)}
    print(f"  R={result['R']} Ω   L={result['L']*1000:.3f} mH   Ke={result['Ke']:.6f} V/(rad/s)")
    return result


# ──────────────────────────────────────────────────────────────────────────────
# Stage 2 — Inertia J and torque constant Kt (from step-response onset)
# ──────────────────────────────────────────────────────────────────────────────
def fit_inertia_kt(datasets: dict, elec: dict) -> dict:
    print("\n── Stage 2: Inertia J and Torque Constant Kt ──")
    key = next((k for k in datasets if "03" in k or "step" in k), None)
    if key is None:
        print("  No step data — using defaults.")
        return {"J": 1e-4, "Kt": 0.05}

    t, vel, cur, tgt = to_arrays(datasets[key])
    vel_rads = vel * 2 * math.pi / 60.0
    cur_A    = cur / 1000.0

    # Find step onset: look for target transitions
    dt = np.median(np.diff(t))

    # Estimate dω/dt at onset of each step → τ = J/B → from J·dω/dt ≈ Kt·I - friction
    # Simple approach: fit first 0.3 s of each step

    # Detect steps in target
    tgt_diff = np.abs(np.diff(tgt))
    step_idxs = np.where(tgt_diff > 20)[0]

    accel_estimates = []
    torque_estimates = []

    for si in step_idxs[:15]:
        onset = si + 1
        horizon = min(onset + int(0.5 / dt), len(vel_rads))
        if horizon - onset < 5:
            continue
        t_w   = t[onset:horizon] - t[onset]
        vel_w = vel_rads[onset:horizon]
        cur_w = cur_A[onset:horizon]
        if len(vel_w) < 4:
            continue
        # Linear fit to velocity rise → dω/dt
        slope = np.polyfit(t_w, vel_w, 1)[0]
        avg_I = cur_w[:int(len(cur_w)*0.5)].mean()
        if abs(slope) > 1 and abs(avg_I) > 0.01:
            accel_estimates.append(slope)
            torque_estimates.append(avg_I)

    if len(accel_estimates) > 2:
        # J·α = Kt·I  →  Kt/J = median(α/I)
        ratio = np.median(np.array(accel_estimates) / np.array(torque_estimates))
        # Assume Kt ≈ Ke (PMDC motor)
        Kt_est = elec.get("Ke", 0.05) * 60 / (2 * math.pi)  # V·s/rad → Nm/A
        J_est  = Kt_est / ratio if abs(ratio) > 0 else 1e-4
    else:
        Kt_est = 0.05
        J_est  = 1e-4

    # Clamp to physically sensible range
    J_est  = float(np.clip(J_est,  1e-6, 1.0))
    Kt_est = float(np.clip(Kt_est, 1e-4, 2.0))

    result = {"J": round(J_est, 8), "Kt": round(Kt_est, 6)}
    print(f"  J={result['J']:.2e} kg·m²   Kt={result['Kt']:.4f} Nm/A")
    return result


# ──────────────────────────────────────────────────────────────────────────────
# Stage 3 — Viscous friction B (from steady-state RPM vs current)
# ──────────────────────────────────────────────────────────────────────────────
def fit_viscous(datasets: dict, elec: dict, mech: dict) -> dict:
    print("\n── Stage 3: Viscous Friction B ──")
    key = next((k for k in datasets if "05" in k or "noise_floor" in k), None)
    if key is None:
        print("  No noise_floor data — using defaults.")
        return {"B": 1e-4}

    t, vel, cur, tgt = to_arrays(datasets[key])
    vel_rads = vel * 2 * math.pi / 60.0
    cur_A    = cur / 1000.0

    # At steady state: Kt·I = B·ω + Fc  →  linear in |ω|
    # Use only segments where |vel| is stable (stdev over 0.5 s window is low)
    omega = np.abs(vel_rads)
    torque = mech.get("Kt", 0.05) * np.abs(cur_A)

    # Filter: only use points where torque > 0.01 (not noise)
    mask = torque > 0.001
    if np.sum(mask) < 10:
        return {"B": 1e-4}

    try:
        coeffs = np.polyfit(omega[mask], torque[mask], 1)
        B_est  = coeffs[0]  # slope = B (viscous)
        Fc_est = coeffs[1]  # intercept ≈ Coulomb torque
    except Exception:
        B_est  = 1e-4
        Fc_est = 0.01

    B_est  = float(np.clip(B_est,  1e-7, 1.0))
    Fc_est = float(np.clip(Fc_est, 0.0,  1.0))

    result = {"B": round(B_est, 8), "Fc_prelim": round(Fc_est, 6)}
    print(f"  B={result['B']:.2e} Nm·s/rad   Fc_prelim={result['Fc_prelim']:.4f} Nm")
    return result


# ──────────────────────────────────────────────────────────────────────────────
# Stage 4 — Coulomb + Stribeck (from slow ramp)
# ──────────────────────────────────────────────────────────────────────────────
def fit_stribeck(datasets: dict, mech: dict, visc: dict) -> dict:
    print("\n── Stage 4: Coulomb + Stribeck Friction ──")
    key = next((k for k in datasets if "04" in k or "slow_ramp" in k), None)
    if key is None:
        print("  No slow_ramp data — using defaults.")
        return {"Fc": 0.02, "Fs": 0.05, "omega_s_rpm": 5.0}

    t, vel, cur, tgt = to_arrays(datasets[key])
    Kt = mech.get("Kt", 0.05)
    B  = visc.get("B", 1e-4)

    omega = vel  # RPM
    torque = Kt * cur / 1000.0  # Nm
    # Remove viscous component
    torque_nv = torque - B * omega * 2 * math.pi / 60.0

    # Only use |ω| > 0.5 to avoid stiction zone dominating
    mask = np.abs(omega) > 0.3
    if np.sum(mask) < 10:
        return {"Fc": 0.02, "Fs": 0.05, "omega_s_rpm": 5.0}

    omega_f  = omega[mask]
    torque_f = torque_nv[mask]

    def stribeck_model(w, Fc, Fs, ws):
        return (Fc * np.sign(w) * (1.0 - np.exp(-np.abs(w) / (ws + 1e-9))) +
                Fs * np.exp(-(w / (ws + 1e-9))**2) * np.sign(w))

    try:
        popt, _ = curve_fit(stribeck_model, omega_f, torque_f,
                             p0=[0.02, 0.05, 5.0],
                             bounds=([0, 0, 0.1], [1.0, 2.0, 200.0]),
                             maxfev=10000)
        Fc_est, Fs_est, ws_est = popt
    except Exception as e:
        print(f"  Stribeck fit failed ({e}) — using viscous prelim")
        Fc_est = visc.get("Fc_prelim", 0.02)
        Fs_est = Fc_est * 2.0
        ws_est = 5.0

    result = {
        "Fc": round(float(np.clip(Fc_est, 0, 2.0)), 6),
        "Fs": round(float(np.clip(Fs_est, 0, 2.0)), 6),
        "omega_s_rpm": round(float(np.clip(ws_est, 0.1, 200.0)), 4),
    }
    print(f"  Fc={result['Fc']:.4f} Nm   Fs={result['Fs']:.4f} Nm   ωs={result['omega_s_rpm']:.2f} RPM")
    return result


# ──────────────────────────────────────────────────────────────────────────────
# Stage 5 — Cogging harmonics (FFT of current at 2 RPM crawl)
# ──────────────────────────────────────────────────────────────────────────────
def fit_cogging(datasets: dict) -> dict:
    print("\n── Stage 5: Cogging Harmonics ──")
    key = next((k for k in datasets if "07" in k or "cogging" in k), None)
    if key is None:
        print("  No cogging data — skipping.")
        return {f"A{i}": 0.0 for i in range(1, 7)} | \
               {f"N{i}": float(i) for i in range(1, 7)} | \
               {f"phi{i}": 0.0 for i in range(1, 7)}

    t, vel, cur, tgt = to_arrays(datasets[key])
    cur_A = cur / 1000.0  # A

    # FFT of current signal → identify dominant harmonics
    dt = float(np.median(np.diff(t)))
    if dt <= 0 or np.isnan(dt):
        dt = 0.01
    freqs = rfftfreq(len(cur_A), d=dt)
    mag   = np.abs(rfft(cur_A - cur_A.mean()))

    # Find 6 largest peaks (excluding DC and very high freq noise)
    mask = (freqs > 0.1) & (freqs < 50.0)
    mag_m = mag.copy()
    mag_m[~mask] = 0

    peaks, props = find_peaks(mag_m, height=mag_m.max() * 0.05)
    # Sort by magnitude
    if len(peaks) > 0:
        sorted_peaks = peaks[np.argsort(mag_m[peaks])[::-1]][:6]
    else:
        sorted_peaks = []

    cogging = {}
    for i in range(1, 7):
        if i - 1 < len(sorted_peaks):
            pi = sorted_peaks[i - 1]
            amplitude = mag[pi] / len(cur_A) * 2.0 * 0.05  # ← rough Nm scaling
            cogging[f"A{i}"]   = round(float(amplitude), 8)
            cogging[f"N{i}"]   = round(float(freqs[pi]), 4)
            cogging[f"phi{i}"] = 0.0
        else:
            cogging[f"A{i}"]   = 0.0
            cogging[f"N{i}"]   = float(i)
            cogging[f"phi{i}"] = 0.0
        print(f"  Harmonic {i}: A={cogging[f'A{i}']:.4e}  f={cogging[f'N{i}']:.2f} Hz")
    return cogging


# ──────────────────────────────────────────────────────────────────────────────
# Stage 6 — Noise model (heteroscedastic: σ = σ0 + σ1·|ω|)
# ──────────────────────────────────────────────────────────────────────────────
def fit_noise(datasets: dict) -> dict:
    print("\n── Stage 6: Noise Model σ(ω) = σ0 + σ1·|ω| ──")
    key = next((k for k in datasets if "05" in k or "noise_floor" in k), None)
    if key is None:
        return {"sigma0": 0.5, "sigma1": 0.002}

    t, vel, cur, tgt = to_arrays(datasets[key])

    # Group by target RPM level, compute stdev of velocity in each group
    unique_tgts = np.unique(np.round(tgt / 10.0) * 10)  # round to nearest 10 RPM
    omegas, stdevs = [], []
    for lvl in unique_tgts:
        mask = np.abs(tgt - lvl) < 20
        if np.sum(mask) > 20:
            stdevs.append(float(np.std(vel[mask])))
            omegas.append(float(abs(lvl)))

    if len(omegas) < 2:
        return {"sigma0": 0.5, "sigma1": 0.002}

    try:
        coeffs = np.polyfit(omegas, stdevs, 1)
        sigma1 = float(np.clip(coeffs[0], 0, 1.0))
        sigma0 = float(np.clip(coeffs[1], 0.01, 50.0))
    except Exception:
        sigma0, sigma1 = 0.5, 0.002

    print(f"  σ0={sigma0:.4f} RPM   σ1={sigma1:.6f} RPM/(RPM)")
    return {"sigma0": round(sigma0, 6), "sigma1": round(sigma1, 8)}


# ──────────────────────────────────────────────────────────────────────────────
# Stage 7 — Dead-zone (min PWM for motion onset)
# ──────────────────────────────────────────────────────────────────────────────
def fit_deadzone(datasets: dict) -> dict:
    print("\n── Stage 7: Dead-zone (PWM onset) ──")
    key = next((k for k in datasets if "06" in k or "deadzone" in k), None)
    if key is None:
        return {"deadzone_pwm_fwd": 300, "deadzone_pwm_rev": 300}

    t, vel, cur, tgt = to_arrays(datasets[key])
    # Look for first sample where |vel| > 2 RPM
    onset_fwd = None
    onset_rev = None
    
    # We only fit reverse if we actually have reverse current data (meaning reverse scan ran)
    has_reverse = np.min(cur) < -20.0
    
    half = len(vel) // 2
    for i in range(half):
        if abs(vel[i]) > 2.0:
            onset_fwd = i
            break
            
    if has_reverse:
        for i in range(half, len(vel)):
            if abs(vel[i]) > 2.0 and vel[i] < -2.0:
                onset_rev = i
                break

    # Map index back to approximate PWM value
    dz_fwd = onset_fwd * 10 if onset_fwd is not None else 300
    dz_rev = (onset_rev - half) * 10 if (onset_rev is not None and has_reverse) else dz_fwd

    print(f"  Dead-zone PWM: fwd={dz_fwd}  rev={dz_rev}")
    return {"deadzone_pwm_fwd": dz_fwd, "deadzone_pwm_rev": dz_rev}


# ──────────────────────────────────────────────────────────────────────────────
# Diagnostic plots
# ──────────────────────────────────────────────────────────────────────────────
def make_plots(datasets: dict, model: dict):
    if not HAS_MPL:
        return
    print("\n── Generating diagnostic plots ──")

    # 1. Step response overlay
    key = next((k for k in datasets if "03" in k), None)
    if key:
        t, vel, cur, tgt = to_arrays(datasets[key])
        fig, axes = plt.subplots(2, 1, figsize=(12, 6), sharex=True)
        axes[0].plot(t, vel, lw=0.6, label="Measured RPM")
        axes[0].plot(t, tgt, lw=1.2, color="red", label="Target RPM")
        axes[0].set_ylabel("RPM"); axes[0].legend(); axes[0].grid(True)
        axes[1].plot(t, cur, lw=0.6, color="orange", label="Current mA")
        axes[1].set_ylabel("Current (mA)"); axes[1].set_xlabel("Time (s)")
        axes[1].legend(); axes[1].grid(True)
        fig.suptitle("Step Response — Measured vs Target")
        fig.tight_layout()
        fig.savefig(REPORT_DIR / "step_response.png", dpi=150)
        plt.close(fig)

    # 2. Frequency response (PRBS)
    key = next((k for k in datasets if "01" in k), None)
    if key:
        t, vel, cur, tgt = to_arrays(datasets[key])
        dt = float(np.median(np.diff(t)))
        if dt > 0:
            from scipy.signal import welch
            f_tgt, P_tgt = welch(tgt, fs=1.0/dt, nperseg=512)
            f_vel, P_vel = welch(vel, fs=1.0/dt, nperseg=512)
            fig, ax = plt.subplots(figsize=(10, 4))
            ax.semilogy(f_tgt, P_tgt, label="Target PSD")
            ax.semilogy(f_vel, P_vel, label="Measured RPM PSD")
            ax.set_xlabel("Frequency (Hz)"); ax.set_ylabel("Power")
            ax.set_title("PRBS Power Spectral Density"); ax.legend(); ax.grid(True)
            fig.tight_layout()
            fig.savefig(REPORT_DIR / "prbs_psd.png", dpi=150)
            plt.close(fig)

    # 3. Friction curve
    key = next((k for k in datasets if "04" in k), None)
    if key:
        t, vel, cur, tgt = to_arrays(datasets[key])
        Kt = model.get("Kt", 0.05)
        torque = Kt * cur / 1000.0
        fig, ax = plt.subplots(figsize=(8, 5))
        sc = ax.scatter(vel, torque, s=1, c=t, cmap="viridis", alpha=0.5)
        plt.colorbar(sc, ax=ax, label="Time (s)")
        ax.set_xlabel("Velocity (RPM)"); ax.set_ylabel("Torque (Nm-equiv)")
        ax.set_title("Friction Torque vs Velocity (Slow Ramp)"); ax.grid(True)
        fig.tight_layout()
        fig.savefig(REPORT_DIR / "friction_curve.png", dpi=150)
        plt.close(fig)

    # 4. Noise floor
    key = next((k for k in datasets if "05" in k), None)
    if key:
        t, vel, cur, tgt = to_arrays(datasets[key])
        unique_tgts = np.unique(np.round(np.abs(tgt) / 10) * 10)
        omegas, stdevs = [], []
        for lvl in unique_tgts:
            mask = np.abs(np.abs(tgt) - lvl) < 20
            if np.sum(mask) > 20:
                stdevs.append(float(np.std(vel[mask])))
                omegas.append(float(lvl))
        fig, ax = plt.subplots(figsize=(7, 4))
        ax.plot(omegas, stdevs, "o-", lw=1.5)
        ax.set_xlabel("Target RPM"); ax.set_ylabel("Velocity Stdev (RPM)")
        ax.set_title("Noise Floor vs Speed"); ax.grid(True)
        sig0 = model.get("sigma0", 0); sig1 = model.get("sigma1", 0)
        ax.plot(omegas, [sig0 + sig1 * w for w in omegas], "--", label=f"σ={sig0:.2f}+{sig1:.4f}·ω")
        ax.legend()
        fig.tight_layout()
        fig.savefig(REPORT_DIR / "noise_floor.png", dpi=150)
        plt.close(fig)

    print(f"  Plots saved to {REPORT_DIR}/")


# ──────────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="Motor Model Fitter")
    parser.add_argument("--data-dir", default="sysid_data", type=pathlib.Path)
    parser.add_argument("--no-plots", action="store_true")
    args = parser.parse_args()

    print("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
    print("  Motor Model Fitter")
    print(f"  Data dir: {args.data_dir}")
    print("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n")

    datasets = load_data(args.data_dir)

    # Staged fitting cascade
    elec     = fit_electrical(datasets)
    mech     = fit_inertia_kt(datasets, elec)
    visc     = fit_viscous(datasets, elec, mech)
    strib    = fit_stribeck(datasets, mech, visc)
    cogging  = fit_cogging(datasets)
    noise    = fit_noise(datasets)
    deadzone = fit_deadzone(datasets)

    model = {
        "version": 1,
        "fitted_at": str(pathlib.Path("sysid_data").resolve()),
        # Electrical
        "R": elec["R"],           # Ω
        "L": elec["L"],           # H
        "Ke": elec["Ke"],         # V·s/rad
        # Mechanical
        "J": mech["J"],           # kg·m²
        "Kt": mech["Kt"],         # Nm/A
        # Friction
        "B":           visc.get("B", 1e-4),
        "Fc":          strib["Fc"],
        "Fs":          strib["Fs"],
        "omega_s_rpm": strib["omega_s_rpm"],
        # Cogging harmonics
        **cogging,
        # Noise
        "sigma0": noise["sigma0"],
        "sigma1": noise["sigma1"],
        # Dead-zone
        "deadzone_pwm_fwd": deadzone["deadzone_pwm_fwd"],
        "deadzone_pwm_rev": deadzone["deadzone_pwm_rev"],
        # System constants
        "max_voltage": 24.0,
    }

    out_path = pathlib.Path("motor_model.json")
    with open(out_path, "w") as f:
        json.dump(model, f, indent=2)

    print(f"\n✅ Model written to: {out_path.resolve()}")
    print(json.dumps(model, indent=2))

    if not args.no_plots:
        make_plots(datasets, model)

    print("\n  Next: launch the sim with  .venv/bin/python mock_sim.py")
    print("  The sim will automatically load motor_model.json")

if __name__ == "__main__":
    main()
