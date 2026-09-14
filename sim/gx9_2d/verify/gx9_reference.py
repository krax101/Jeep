#!/usr/bin/env python3
"""Standalone reference implementation of the GX9 2-D performance model.

This is NOT the deliverable -- GX9_2D.slx is.  It exists so the equations
that go into the Simulink model can be executed, checked for sanity and
regression-tested on a machine that has no MATLAB licence (CI, a review
box, a laptop).  It reads the very same gx9_params.json, and implements the
very same equations as the MATLAB Function blocks in blockfcn/, so the two
must agree.  gx9_selftest.m checks the Simulink model against the metrics
this script writes to expected_metrics.json.

Standard library only: no numpy, no scipy.

Usage:
    python3 gx9_reference.py              # run scenarios, print a report
    python3 gx9_reference.py --write      # also refresh expected_metrics.json
"""

import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PARAM_FILE = os.path.join(HERE, os.pardir, "gx9_params.json")


# ---------------------------------------------------------------------
# Parameters: mirrors gx9_params.m
# ---------------------------------------------------------------------
def solve_rs(rp, C, L, rs0=0.08):
    """Secondary pitch radius holding belt length at L (Newton, see gx9_params.m)."""
    rs = rs0
    for _ in range(60):
        d = rp - rs
        beta = math.asin(max(-0.999, min(0.999, d / C)))
        f = math.pi * (rp + rs) + 2 * beta * d + 2 * C * math.cos(beta) - L
        df = math.pi - 2 * beta
        step = f / df
        rs -= step
        if abs(step) < 1e-15:
            break
    return rs


def load_params(path=PARAM_FILE):
    J = json.load(open(path))
    P = {}

    P["rho"] = J["environment"]["air_density_kgm3"]
    P["g"] = J["environment"]["gravity_mps2"]

    e = J["engine"]
    P["eng_w"] = [r * math.pi / 30 for r in e["rpm_brk"]]
    P["eng_T"] = list(e["torque_wot_Nm"])
    P["w_idle"] = e["idle_rpm"] * math.pi / 30
    P["w_gov"] = e["governor_rpm"] * math.pi / 30
    P["w_droop"] = e["governor_droop_rpm"] * math.pi / 30
    P["J_e"] = e["inertia_kgm2"]

    c = J["cvt"]
    P["C"] = c["center_distance_m"]
    P["L"] = c["belt_pitch_length_m"]
    P["rp_min"] = c["rp_min_m"]
    P["rp_max"] = c["rp_max_m"]
    P["alpha"] = math.radians(c["sheave_half_angle_deg"])
    P["mw"] = c["flyweight_mass_total_kg"]
    P["Rw"] = c["flyweight_radius_m"]
    P["ramp_x"] = list(c["ramp_travel_norm_brk"])
    P["ramp_a"] = list(c["ramp_advantage"])
    P["kp"] = c["primary_spring_rate_Npm"]
    P["Fp0"] = c["primary_spring_preload_N"]
    P["ks"] = c["secondary_spring_rate_Npm"]
    P["Fs0"] = c["secondary_spring_preload_N"]
    P["Rh"] = c["helix_radius_m"]
    P["th"] = math.radians(c["helix_angle_deg"])
    P["tau_T"] = c["torque_sense_lag_s"]
    P["mu_b"] = c["belt_friction_coeff"]
    P["k_slip"] = c["belt_slip_stiffness_Nms_per_rad"]
    P["F_eng_ref"] = c["engagement_clamp_ref_N"]
    P["c_up"] = c["shift_damping_up_Nspm"]
    P["c_dn"] = c["shift_damping_down_Nspm"]
    P["eff_r"] = list(c["eff_ratio_brk"])
    P["eff_T"] = list(c["eff_torque_brk"])
    P["eff_tab"] = [list(row) for row in c["eff_table"]]

    n = 201
    P["rp_brk"] = [P["rp_min"] + (P["rp_max"] - P["rp_min"]) * k / (n - 1) for k in range(n)]
    rs_tab, guess = [], 0.08
    for rp in P["rp_brk"]:
        guess = solve_rs(rp, P["C"], P["L"], guess)
        rs_tab.append(guess)
    P["rs_tab"] = rs_tab
    P["rs_max"], P["rs_min"] = rs_tab[0], rs_tab[-1]
    P["ratio_low"] = P["rs_max"] / P["rp_min"]
    P["ratio_high"] = P["rs_min"] / P["rp_max"]
    P["tw"] = 2 * math.tan(P["alpha"])
    P["xp_max"] = P["tw"] * (P["rp_max"] - P["rp_min"])
    P["xs_max"] = P["tw"] * (P["rs_max"] - P["rs_min"])
    P["w_engage"] = math.sqrt(P["Fp0"] / (P["mw"] * P["Rw"] * P["ramp_a"][0]))
    P["rpm_engage"] = P["w_engage"] * 30 / math.pi

    d = J["driveline"]
    P["i_gb"] = d["gearbox_ratio"]
    P["eta_gb"] = d["gearbox_efficiency"]
    P["J_sec"] = d["secondary_side_inertia_kgm2"]

    v = J["vehicle"]
    P["m"] = v["mass_vehicle_kg"] + v["mass_driver_kg"]
    P["Lwb"] = v["wheelbase_m"]
    P["a"] = v["cg_to_front_axle_m"]
    P["b"] = P["Lwb"] - P["a"]
    P["h"] = v["cg_height_m"]
    P["rt"] = v["tire_radius_m"]
    P["Jw"] = v["wheel_inertia_total_kgm2"]
    P["mu_t"] = v["tire_friction_coeff"]
    P["Crr"] = v["rolling_resistance_coeff"]
    P["CdA"] = v["drag_coefficient"] * v["frontal_area_m2"]
    P["m_eff"] = P["m"] + (P["Jw"] + P["J_sec"] * P["i_gb"] ** 2) / P["rt"] ** 2

    s = J["scenario"]
    P["grade"] = math.radians(s["grade_deg"])
    P["throttle"] = s["throttle"]
    P["t_end"] = s["sim_time_s"]
    P["accel_m"] = s["accel_distance_m"]
    return P


def interp(xb, yb, x):
    if x <= xb[0]:
        return yb[0]
    if x >= xb[-1]:
        return yb[-1]
    lo, hi = 0, len(xb) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if x < xb[mid]:
            hi = mid
        else:
            lo = mid
    t = (x - xb[lo]) / (xb[hi] - xb[lo])
    return yb[lo] + t * (yb[hi] - yb[lo])


def interp2(xb, yb, tab, x, y):
    """Bilinear, clipped -- mirrors the 2-D Lookup Table block."""
    x = max(xb[0], min(xb[-1], x))
    y = max(yb[0], min(yb[-1], y))
    i = max(0, min(len(xb) - 2, sum(1 for v in xb if v <= x) - 1))
    j = max(0, min(len(yb) - 2, sum(1 for v in yb if v <= y) - 1))
    tx = (x - xb[i]) / (xb[i + 1] - xb[i])
    ty = (y - yb[j]) / (yb[j + 1] - yb[j])
    return ((1 - tx) * (1 - ty) * tab[i][j] + tx * (1 - ty) * tab[i + 1][j]
            + (1 - tx) * ty * tab[i][j + 1] + tx * ty * tab[i + 1][j + 1])


# ---------------------------------------------------------------------
# Block equations: mirror blockfcn/*.m one for one
# ---------------------------------------------------------------------
def cvt_geom(rp, P):
    rs = interp(P["rp_brk"], P["rs_tab"], rp)
    beta = math.asin(max(-0.999, min(0.999, (rp - rs) / P["C"])))
    theta_p = math.pi + 2 * beta
    theta_s = math.pi - 2 * beta
    return (rs, rs / max(rp, 1e-6), theta_p, theta_s, theta_p / theta_s,
            P["tw"] * (rp - P["rp_min"]), P["tw"] * (rs - P["rs_min"]))


def cvt_forces(w_e, T_sec, rp, xp, xs, lever, theta_p, P):
    phi = interp(P["ramp_x"], P["ramp_a"], xp / max(P["xp_max"], 1e-9))
    Fp = P["mw"] * w_e ** 2 * P["Rw"] * phi - (P["Fp0"] + P["kp"] * xp)
    Fs = P["Fs0"] + P["ks"] * xs + abs(T_sec) / (P["Rh"] * math.tan(P["th"]))
    F_shift = Fp - lever * Fs
    clamp = max(Fp, 0.0)
    engage = max(0.0, min(1.0, clamp / P["F_eng_ref"]))
    c = P["c_up"] if F_shift >= 0 else P["c_dn"]
    rp_dot = engage * F_shift / c
    sa = math.sin(P["alpha"])
    wrap = 1 - math.exp(-(P["mu_b"] / sa) * max(theta_p, 0.0))
    T_cap = P["mu_b"] * clamp * rp / sa * wrap
    return Fp, Fs, F_shift, rp_dot, T_cap, engage


def belt(w_e, w_sec, ratio, T_cap, engage, P):
    slip = w_e - ratio * w_sec
    return max(-T_cap, min(T_cap, P["k_slip"] * slip)) * engage, slip


def governor(thr_cmd, w_e, P):
    f = max(0.0, min(1.0, 1 - (w_e - P["w_gov"]) / P["w_droop"]))
    return max(0.0, min(1.0, thr_cmd)) * f


def vehicle_2d(F_dem, v, grade, P):
    g, m, L, h, a, b, mu = P["g"], P["m"], P["Lwb"], P["h"], P["a"], P["b"], P["mu_t"]
    cth, sth = math.cos(grade), math.sin(grade)
    F_res = (0.5 * P["rho"] * P["CdA"] * v * abs(v)
             + P["Crr"] * m * g * cth * math.tanh(v / 0.30)
             + m * g * sth)
    ax_free = (F_dem - F_res) / P["m_eff"]
    Nr_free = (m * g * cth * a + m * ax_free * h + m * g * sth * h) / L
    F_max = mu * max(Nr_free, 0.0)
    if F_dem <= F_max:
        tract_lim, ax, F_trac = 0, ax_free, F_dem
    else:
        tract_lim = 1
        den = P["m_eff"] - mu * m * h / L
        ax = ((mu * m * g * (cth * a + sth * h) / L - F_res) / den) if den > 1e-6 else ax_free
        F_trac = P["m_eff"] * ax + F_res
    Nf = (m * g * cth * b - m * ax * h - m * g * sth * h) / L
    Nr = (m * g * cth * a + m * ax * h + m * g * sth * h) / L
    return ax, F_trac, Nf, Nr, F_res, tract_lim


# ---------------------------------------------------------------------
# Integration
# ---------------------------------------------------------------------
def derivs(x, u, P):
    w_e, rp, v, dist, T_sec_f = x
    thr_cmd, grade = u

    rp = max(P["rp_min"], min(P["rp_max"], rp))
    rs, ratio, th_p, th_s, lever, xp, xs = cvt_geom(rp, P)

    w_wheel = v / P["rt"]
    w_sec = w_wheel * P["i_gb"]

    Fp, Fs, F_shift, rp_dot, T_cap, engage = cvt_forces(
        w_e, T_sec_f, rp, xp, xs, lever, th_p, P)

    T_pri, slip = belt(w_e, w_sec, ratio, T_cap, engage, P)
    T_sec_ideal = T_pri * ratio
    eta = interp2(P["eff_r"], P["eff_T"], P["eff_tab"], ratio, abs(T_sec_ideal))
    T_sec = T_sec_ideal * eta
    T_sec_f_dot = (T_sec - T_sec_f) / P["tau_T"]

    T_e = interp(P["eng_w"], P["eng_T"], w_e) * governor(thr_cmd, w_e, P)
    w_dot = (T_e - T_pri) / P["J_e"]
    if w_e <= 0.0 and w_dot < 0.0:
        w_dot = 0.0

    F_dem = T_sec * P["i_gb"] * P["eta_gb"] / P["rt"]
    ax, F_trac, Nf, Nr, F_res, tract_lim = vehicle_2d(F_dem, v, grade, P)
    if v <= 0.0 and ax < 0.0:
        ax = 0.0

    if (rp >= P["rp_max"] and rp_dot > 0) or (rp <= P["rp_min"] and rp_dot < 0):
        rp_dot = 0.0

    diag = dict(rs=rs, ratio=ratio, Fp=Fp, Fs=Fs, F_shift=F_shift, T_cap=T_cap,
                engage=engage, slip=slip, eta=eta, T_e=T_e, T_pri=T_pri,
                T_sec=T_sec, ax=ax, F_trac=F_trac, F_res=F_res, Nf=Nf, Nr=Nr,
                tract_lim=tract_lim, rpm=w_e * 30 / math.pi,
                w_sec=w_sec, kmh=v * 3.6)
    return [w_dot, rp_dot, ax, v, T_sec_f_dot], diag


def simulate(P, t_end=None, dt=2e-4, throttle=None, grade=None, log_every=25):
    t_end = P["t_end"] if t_end is None else t_end
    thr = P["throttle"] if throttle is None else throttle
    grd = P["grade"] if grade is None else grade

    x = [P["w_idle"], P["rp_min"], 0.0, 0.0, 0.0]
    n = int(round(t_end / dt))
    hist = {k: [] for k in ("t", "rpm", "v", "kmh", "dist", "ratio", "rp", "F_shift",
                            "T_cap", "slip", "engage", "ax", "Nf", "Nr", "tract_lim",
                            "T_sec", "eta", "F_trac")}
    for k in range(n + 1):
        t = k * dt
        d, diag = derivs(x, (thr, grd), P)
        if k % log_every == 0:
            hist["t"].append(t)
            hist["v"].append(x[2])
            hist["dist"].append(x[3])
            hist["rp"].append(x[1])
            for key in ("rpm", "kmh", "ratio", "F_shift", "T_cap", "slip", "engage",
                        "ax", "Nf", "Nr", "tract_lim", "T_sec", "eta", "F_trac"):
                hist[key].append(diag[key])
        if k == n:
            break
        # Explicit Euler at a small fixed step. The fastest modes are the
        # belt-slip coupling on engine speed (tau = J_e/k_slip ~ 1.3 ms) and
        # the torque-sense lag (5 ms), so 0.2 ms is comfortably inside the
        # stability limit of both.
        for i in range(len(x)):
            x[i] += dt * d[i]
        x[1] = max(P["rp_min"], min(P["rp_max"], x[1]))
        x[2] = max(0.0, x[2])
        x[0] = max(0.0, x[0])
    return hist


def crossing_time(hist, key, value):
    xs, ts = hist[key], hist["t"]
    for i in range(1, len(xs)):
        if xs[i - 1] < value <= xs[i]:
            f = (value - xs[i - 1]) / (xs[i] - xs[i - 1])
            return ts[i - 1] + f * (ts[i] - ts[i - 1])
    return None


def gradeability(P, max_deg=45.0, step_deg=0.5, t=12.0):
    """Steepest grade the car will still crawl up at >0.5 m/s."""
    best = 0.0
    deg = 0.0
    while deg <= max_deg:
        h = simulate(P, t_end=t, grade=math.radians(deg), log_every=200)
        if h["v"][-1] > 0.5:
            best = deg
        else:
            break
        deg += step_deg
    return best


def metrics(P):
    acc = simulate(P, t_end=12.0)
    top = simulate(P, t_end=40.0)

    def at_end(h, key):
        return h[key][-1]

    shift_start = next((t for t, r in zip(acc["t"], acc["ratio"])
                        if r < P["ratio_low"] - 0.05), None)
    locked = [rpm for rpm, e, s in zip(acc["rpm"], acc["engage"], acc["slip"])
              if e >= 0.999 and abs(s) < 5.0]
    return {
        "ratio_low": P["ratio_low"],
        "ratio_high": P["ratio_high"],
        "rpm_engage": P["rpm_engage"],
        "xp_max_mm": P["xp_max"] * 1e3,
        "xs_max_mm": P["xs_max"] * 1e3,
        "m_eff_kg": P["m_eff"],
        "t_100ft_s": crossing_time(acc, "dist", 30.48),
        "t_150ft_s": crossing_time(acc, "dist", P["accel_m"]),
        "t_0_30kmh_s": crossing_time(acc, "kmh", 30.0),
        "t_0_50kmh_s": crossing_time(acc, "kmh", 50.0),
        "top_speed_kmh": max(top["kmh"]),
        "rpm_at_top_speed": at_end(top, "rpm"),
        "ratio_at_top_speed": at_end(top, "ratio"),
        "shift_start_s": shift_start,
        "mean_locked_rpm": (sum(locked) / len(locked)) if locked else None,
        "max_rpm": max(acc["rpm"]),
        "peak_ax_mps2": max(acc["ax"]),
        "min_front_load_N": min(acc["Nf"]),
        "traction_limited_frac": sum(acc["tract_lim"]) / len(acc["tract_lim"]),
        "max_belt_slip_radps": max(abs(s) for s in acc["slip"]),
        "gradeability_deg": gradeability(P),
    }


def main():
    P = load_params()
    M = metrics(P)

    print("=" * 66)
    print("  Gaged GX9 - 2-D performance model, reference solver")
    print("=" * 66)
    print("\n[ derived hardware geometry ]")
    print(f"  ratio range            {M['ratio_low']:.3f}:1  ->  {M['ratio_high']:.3f}:1")
    print(f"  primary pitch radius   {P['rp_min']*1e3:.1f} -> {P['rp_max']*1e3:.1f} mm"
          f"   (sheave travel {M['xp_max_mm']:.1f} mm)")
    print(f"  secondary pitch radius {P['rs_min']*1e3:.1f} -> {P['rs_max']*1e3:.1f} mm"
          f"   (sheave travel {M['xs_max_mm']:.1f} mm)")
    print(f"  engagement speed       {M['rpm_engage']:.0f} rpm")
    print(f"  effective mass         {M['m_eff_kg']:.1f} kg"
          f"   (kerb+driver {P['m']:.0f} kg)")

    print("\n[ wide-open-throttle acceleration, flat ]")
    print(f"  0 -> 30 km/h           {M['t_0_30kmh_s']:.2f} s")
    print(f"  0 -> 50 km/h           {M['t_0_50kmh_s']:.2f} s")
    print(f"  100 ft (30.5 m)        {M['t_100ft_s']:.2f} s")
    print(f"  150 ft (45.7 m)        {M['t_150ft_s']:.2f} s")
    print(f"  peak acceleration      {M['peak_ax_mps2']:.2f} m/s^2"
          f"  ({M['peak_ax_mps2']/9.81:.2f} g)")
    print(f"  traction limited for   {100*M['traction_limited_frac']:.0f}% of the run")
    print(f"  min front axle load    {M['min_front_load_N']:.0f} N")

    print("\n[ CVT behaviour ]")
    print(f"  upshift begins at      {M['shift_start_s']:.2f} s")
    print(f"  held engine speed      {M['mean_locked_rpm']:.0f} rpm (mean, belt locked)")
    print(f"  peak engine speed      {M['max_rpm']:.0f} rpm")
    print(f"  peak belt slip         {M['max_belt_slip_radps']:.1f} rad/s")

    print("\n[ steady state ]")
    print(f"  top speed              {M['top_speed_kmh']:.1f} km/h"
          f"  at {M['rpm_at_top_speed']:.0f} rpm, ratio {M['ratio_at_top_speed']:.2f}:1")
    print(f"  max gradeability       {M['gradeability_deg']:.1f} deg"
          f"  ({100*math.tan(math.radians(M['gradeability_deg'])):.0f}% grade)")
    print()

    if "--write" in sys.argv:
        out = os.path.join(HERE, "expected_metrics.json")
        json.dump(M, open(out, "w"), indent=2, sort_keys=True)
        print(f"wrote {out}")
    return M


if __name__ == "__main__":
    main()
