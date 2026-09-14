# Gaged GX9 — 2-D performance model (Simulink)

A Simulink model that predicts what a [Gaged Engineering GX9](https://www.gagedengineering.com/gx9-specifications/)
CVT will do in a Baja SAE car: launch, shift curve, acceleration times, top
speed and gradeability.

## What "2-D" means here

Two things, and the model does both:

1. **The vehicle is planar.** Longitudinal motion *plus* the pitch moment it
   creates, so normal load transfers off the front axle onto the driven rear
   axle as the car accelerates. On a short-wheelbase, high-CG buggy this is
   not a detail — the rear grip that limits a launch is itself a function of
   the acceleration being predicted. Solved in closed form, not iterated.

2. **The CVT is treated as a 2-D sheave/belt geometry**, not as a ratio
   lookup. Belt pitch radius and sheave axial position are separate,
   physically-coupled coordinates on each unit, and the two units are locked
   together by the belt length. The ratio is an *output* of a force balance,
   not an input.

If you only wanted a ratio-vs-rpm map, this is more model than you need. It
earns its keep when you want to know *why* the shift curve sits where it does
and what changing a flyweight or a helix angle will do to it.

## Layout

```
gx9_params.json           every primitive number, single source of truth
gx9_params.m              loads the JSON, derives geometry, returns struct P
build_gx9_2d_model.m      builds GX9_2D.slx programmatically   <- the model
blockfcn/*.m              the MATLAB Function block bodies, as reviewable files
run_gx9_2d.m              scenarios, metrics, plots
gx9_postprocess.m         logged bus -> named signals + performance metrics
gx9_sig_names.m           channel map of the logged bus
gx9_sweep.m               sweep a tuning parameter, tabulate the effect
gx9_selftest.m            model vs reference solver + physical invariants
gx9_refresh_params.m      push edited JSON into an already-built model
verify/gx9_reference.py   same equations in plain Python (no MATLAB needed)
verify/expected_metrics.json  baseline the self-test compares against
```

The model is **built from source** rather than committed as a binary `.slx`,
so it reviews, diffs and merges like the rest of this repo.

## Running it

```matlab
cd sim/gx9_2d
build_gx9_2d_model          % writes and opens GX9_2D.slx
run_gx9_2d                  % acceleration + top speed + gradeability + plots
gx9_selftest                % check the model against the reference solver
```

Other entry points:

```matlab
run_gx9_2d('Scenario','accel')            % acceleration only
run_gx9_2d('Scenario','grade','Grade',20) % hold a 20 degree climb
run_gx9_2d('Throttle',0.5)                % part throttle
gx9_sweep('cvt.mw', 0.18:0.02:0.30)       % flyweight mass sweep
gx9_sweep('cvt.Fp0', 700:100:1200)        % engagement RPM sweep
```

Requires Simulink (R2018b or newer — the plots use `yline`/`xline`). No
toolboxes beyond base Simulink. `verify/gx9_reference.py` needs only Python 3
standard library, so the physics can be exercised with no MATLAB at all.

## Model structure

```
GX9_2D
 |- Engine        WOT torque map + droop governor, engine speed state
 |- CVT_GX9       belt geometry, sheave force balance, shift state,
 |                slipping-belt coupling, loss map
 |- Driveline     gearbox reduction and efficiency -> tractive force demand
 |- Vehicle_2D    planar dynamics: longitudinal + pitch load transfer
```

Four continuous states: engine speed, primary pitch radius (the shift state),
vehicle speed, distance. A fifth small state carries the torque-sense lag.

### The CVT

**Belt geometry.** With pitch radii `rp`, `rs` and centre distance `C`:

```
beta    = asin((rp - rs)/C)          (signed, so it works either side of 1:1)
theta_p = pi + 2*beta                 wrap angle on the primary
theta_s = pi - 2*beta                 wrap angle on the secondary
L       = pi*(rp+rs) + 2*beta*(rp-rs) + 2*C*cos(beta)
```

Differentiating `L = const` gives, exactly,

```
drs/drp = -(pi + 2*beta)/(pi - 2*beta) = -theta_p/theta_s
```

The shift lever between the two units is just the ratio of their wrap angles.
`rs(rp)` is pre-solved by Newton iteration in `gx9_params.m` and looked up in
the model, so no block iterates at run time. Worst belt-length error across
the full shift range is 0.2 µm.

**Force balance.** Primary: flyweights against a ramp, opposed by the primary
spring. Secondary: compression spring plus a torque-sensing helix, which is
what makes the unit backshift under load.

```
F_p = m_w * w_e^2 * R_w * phi(x_p) - (Fp0 + kp*x_p)
F_s = Fs0 + ks*x_s + |T_sec| / (R_h * tan(theta_h))
```

Virtual work on a belt-length-preserving shift (`dx = 2*tan(alpha)*dr` on each
unit, with `drs/drp` as above) collapses both axial forces onto one axis:

```
F_shift = F_p - (theta_p/theta_s) * F_s
```

Positive upshifts, negative backshifts, and the shift rate is damping-limited
— faster down than up, as on real rubber-belt units.

**The belt is not a rigid ratio constraint.** Below engagement the primary
does not grip and the engine free-revs; just above it the clamp force and
therefore the friction capacity are small, so the car launches on a slipping
belt. Once clamped, slip collapses to a fraction of a rad/s. This is what
makes the launch prediction worth anything.

### The vehicle

```
Nf = [ m*g*cos(th)*b - m*ax*h - m*g*sin(th)*h ] / L
Nr = [ m*g*cos(th)*a + m*ax*h + m*g*sin(th)*h ] / L
```

`ax` appears on both sides. Rather than iterate, the traction-limited branch
is solved in closed form:

```
ax = [ mu*m*g*(cos(th)*a + sin(th)*h)/L - F_res ] / (m_eff - mu*m*h/L)
```

The denominator is the acceleration feedback through load transfer. It going
non-positive means rear grip is growing faster than the inertia resisting it —
the car is lifting its front wheels — which is trapped and reported through
`Nf` rather than divided by.

## Predictions with the shipped parameter set

Baja-spec Briggs & Stratton Model 19, 277 kg with driver, 8.5:1 gearbox,
0.286 m tyres, dry hardpack (`mu = 0.8`):

| | |
|---|---|
| ratio range | 3.90:1 → 0.90:1 |
| engagement | 2004 rpm |
| primary sheave travel | 20.3 mm |
| 100 ft | 4.39 s |
| **150 ft (Baja acceleration event)** | **5.68 s** |
| 0 → 30 km/h | 2.54 s |
| peak acceleration | 5.44 m/s² (0.55 g) |
| min front axle load | 571 N (of 1143 N static) |
| held engine speed | 3525 rpm |
| top speed | 54.7 km/h at 3887 rpm, fully upshifted |
| gradeability | 31.5° (61%) |

Top speed is governor-limited, not gearing-limited — correct for a Baja car.

## Calibrating it to your car

**Read this before quoting any number above.** The published GX9 facts are the
3.9:1–0.9:1 ratio range and the unit envelope dimensions; the centre distance,
pitch radii and belt length are *derived* to reproduce that range inside that
envelope, and the springs, flyweights and helix are *nominal*. Every value in
`gx9_params.json` carries a `_provenance` tag saying which it is.

In rough order of how much they move the answer:

1. **Engine curve** (`engine.rpm_brk` / `torque_wot_Nm`) — replace with your
   own dyno sheet. Everything downstream scales off it.
2. **Vehicle mass, CG height and `cg_to_front_axle_m`** — weigh the car on
   corner scales with the driver in it. CG height drives the whole load
   transfer story.
3. **`tire_friction_coeff`** — this single number sets whether the launch is
   grip-limited or power-limited. Measure it, don't guess it.
4. **Centre distance, `rp_min`/`rp_max`, belt length** — measure the actual
   unit. `gx9_params.m` prints the achieved ratio range; if it does not come
   out at 3.9/0.9 your three numbers are inconsistent with each other.
5. **Flyweight mass, ramp profile, spring rates, helix angle** — these set the
   shift curve. Tune them against a logged RPM-vs-speed trace, not in the
   abstract. `gx9_sweep` exists for exactly this.

The two knobs that do what you expect:

- `cvt.Fp0` (primary preload) sets **engagement RPM**, directly and alone.
- `cvt.ramp_advantage` and `cvt.mw` set **where the shift curve sits**. Raise
  them and the CVT holds a higher engine speed.

After editing the JSON, either rebuild (`build_gx9_2d_model`) or push the
change into the loaded model with `gx9_refresh_params`. The breakpoint tables
are baked into the model workspace at build time, so editing the JSON alone
changes nothing.

## Verification

`gx9_selftest` checks the Simulink model against `verify/gx9_reference.py` —
the same equations in plain Python, fixed-step explicit Euler — to 3%, and
then checks physical invariants that must hold regardless of parameters:

- belt length constant to 1 µm through the whole shift
- ratio never leaves the hardware range
- axle loads sum to `m*g`
- neither axle load goes negative
- efficiency stays in (0, 1]
- transmitted torque never exceeds friction capacity
- engine speed stays inside the governor band

The reference solver is step-size converged: 150 ft time changes by 0.1 ms
between dt = 5e-4 and dt = 5e-5.

Regenerate the baseline after a deliberate parameter change:

```
python3 verify/gx9_reference.py --write
```

## Known limitations

- **Longitudinal only.** No lateral dynamics, no yaw, no suspension travel.
  Load transfer is quasi-static about a rigid chassis. Good for straight-line
  events and hill climbs; not a handling model.
- **No tyre slip model.** Grip is a hard `mu*N` ceiling. Once the ceiling is
  hit the model reports the limit rather than simulating wheelspin, so launch
  times on very low `mu` are optimistic.
- **Single drive axle.** A 4WD car with a locked centre would transfer less
  load penalty; this model would under-predict its launch.
- **Aero and rolling resistance act at the CG.** Real aero drag acts higher,
  which adds a small extra pitch moment.
- **Belt efficiency is a map, not a model.** Slip losses under gross slip are
  not separately accounted for beyond the capacity limit.
- **The engine is quasi-static.** A torque map times a governor factor — no
  manifold filling, no thermal effects.

## Sources

- [Gaged Engineering — GX9 specifications](https://www.gagedengineering.com/gx9-specifications/)
- [Gaged Engineering — GX9 systems](https://www.gagedengineering.com/gx9-systems/)
- [Briggs & Stratton Model 20 racing engine](https://www.briggsandstratton.com/na/en_us/product-catalog/engines/racing-engines/model-20.html)
- [Baja SAE](https://en.wikipedia.org/wiki/Baja_SAE)
