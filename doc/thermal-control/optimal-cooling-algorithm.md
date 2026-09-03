# The Optimal SONiC Cooling Algorithm (single algorithm)

Goal: run **one** fan-control algorithm inside `thermalctld` — not a vendor-selectable
superset. The algorithm is a **hardened incremental (velocity-form) PID**. It is
multi-stage in the sense that each tick runs through a fixed sequence of phases, but
there is **only one algorithm and one code path** — no per-vendor mode switching.

This is the algorithm that was actually deployed and **measured** on hardware
(see `08_power_savings_measured.md`), not a paper design.

---

## 0. PID in one page (background)

**PID = Proportional-Integral-Derivative** — the standard feedback control algorithm for
holding a measured value at a target (setpoint) by continuously adjusting an output.
Here the measured value is **temperature** and the output is **fan PWM**.

Each tick it computes the **error** = `measured - target` (e.g. `T - target_temp`),
then sums three terms:

- **P — Proportional** (`kp * error`): react to how far off you are *now*. Bigger error
  -> bigger push. P alone never fully reaches target; it settles with a small standing
  offset (droop).
- **I — Integral** (`ki * sum(error)`): react to how *long* you've been off. It
  accumulates past error and keeps pushing until the leftover offset is driven to zero —
  this is what lets fans settle at *exactly* the speed needed to hold target (the
  steady-state power saver). Risk: **wind-up** if the output saturates.
- **D — Derivative** (`kd * rate_of_change_of_error`): react to how *fast* it's changing.
  Anticipates and damps overshoot/oscillation. Risk: it **amplifies sensor noise**, which
  is why we feed it EMA-smoothed input.

```
output = kp*error + ki*integral + kd*derivative
```

`kp, ki, kd` are the tunable gains (this design: `kp=0.075, ki=1, kd=10`).

**Two forms:**
- *Positional*: compute the absolute output each tick (needs an explicit integral you
  must clamp against wind-up).
- *Incremental / velocity form* (**what we use**): compute a *change* to the output each
  tick and add it:
  ```
  dPWM = kp*(T - T_prev) + ki*error + kd*(T - 2*T_prev + T_prev2)
  pwm  = pwm + dPWM
  ```
  The integral is implicit (you accumulate deltas onto `pwm`), so wind-up is naturally
  bounded and the loop re-seeds cleanly after a reboot — exactly why it fits fan control.

**Intuition:** P = "how far off," I = "how long off," D = "how fast changing." P gets you
close, I removes the leftover gap, D keeps it from overshooting.

**EMA (Exponential Moving Average)** smooths the noisy temperature signal before it enters
the loop: `avg = avg + (T - avg)/N` (larger `N` = heavier smoothing). Without it, the D
term turns sensor flutter into fan hunting.

---

## 1. Why this one

- It is the only configuration in this analysis with **measured** savings on real
  hardware (Arista DCS-7060X6-64PE, Broadcom TH5):
  about **-16.6 W** matched-ambient (~1.9% of total switch power), peak fan
  **82% -> 49%**, optics ~1.4 C cooler, **no thermal penalty**.
- **Velocity (incremental) form** computes a PWM *delta* each tick and accumulates it,
  so there is no explicit integrator to wind up: anti-windup is nearly free and
  warm/fast-reboot re-seeding is trivial (start from the current measured PWM).
- The "enhancements" (deadband, filtered-D, slew limit, critical-margin clamp,
  fail-safe-to-full, soft cap) are **guards inside the one loop**, not separate
  algorithms. They are what turned raw PID from "can hurt on noisy sensors" into the
  measured win.

---

## 2. Where it runs

`thermalctld` already delegates to a pluggable thermal manager and calls it every tick:

```
sonic-thermalctld/scripts/thermalctld
  1472  self.thermal_manager = self.chassis.get_thermal_manager()
  1474      self.thermal_manager.initialize()
  1475      self.thermal_manager.load('/usr/share/sonic/platform/thermal_policy.json')
  1476      self.thermal_manager.init_thermal_algorithm(self.chassis)
  1557      self.thermal_manager.run_policy(self.chassis)      # <-- the tick
```

The algorithm lives once behind `run_policy`. No new daemon plumbing is needed.

---

## 3. Inputs / outputs

Inputs (per tick), all already in `sonic-platform-common`:
- `chassis.get_all_thermals()` -> per sensor `get_temperature()`,
  `get_high_threshold()`, `get_high_critical_threshold()`, `get_name()`
- `chassis.get_all_fans()` -> `get_presence()`, `get_status()`, `get_speed()`,
  `get_target_speed()`

Output (per tick):
- `fan.set_speed(pwm_percent)` for each controllable fan

State carried between ticks: `pwm`, `T_prev`, `T_prev2`, `d_filtered`.

---

## 4. The algorithm — one loop, fixed phases

Pick the **worst (most critical) sensor** in the zone each tick, then run these phases.
This is the whole algorithm; there is no alternate path.

```
# Phase 1 - read + smooth
T      = ema(read(sensor), T_ema, ema_n)          # input smoothing kills D-noise
error  = T - (target + targetOffset)

# Phase 2 - deadband (anti-hunt): freeze inside the band
if -negHyst < error < posHyst:
    pwm_hold = True
else:
    pwm_hold = False

# Phase 3 - incremental PID (velocity form)
d_raw      = T - 2*T_prev + T_prev2               # second difference = dD/dt
d_filtered = alpha*d_raw + (1-alpha)*d_filtered   # filtered derivative
dP         = kp*(T - T_prev) + ki*error + kd*d_filtered
if pwm_hold: dP = 0

# Phase 4 - slew limit (bound how fast PWM can move per tick)
dP  = clamp(dP, -slew_down, +slew_up)
pwm = pwm + dP

# Phase 5 - bounds + soft cap (soft cap is the steady-state power saver)
pwm = clamp(pwm, minSpeed, normalMaxSpeed_softcap)

# Phase 6 - safety overrides (always win, in order)
if (Tcrit - T) < critical_margin:   pwm = ramp_to_full   # critical-margin clamp
if sensor_invalid or fan_fault:     pwm = 100            # fail-safe to full

# commit
fan.set_speed(pwm)
T_prev2, T_prev = T_prev, T
```

Notes:
- Phases 4-6 are **guards**, not stages you can swap out. They are always active.
- Multi-fan / multi-zone: run one loop per zone on that zone's worst sensor; a fan
  fault in any zone forces that zone to 100%.
- Safety overrides are unconditional and ordered last so they cannot be tuned away.

---

## 5. Config (one JSON per SKU, no logic in platform code)

Only **tuning data** lives in `/usr/share/sonic/platform/thermal_policy.json`. The
control code is identical everywhere.

```jsonc
{
  "interval": 15,
  "zones": [{
    "name": "main",
    "fans": ["Fan1..N"],
    "sensor_select": "worst",
    "pid":   { "kp": 0.075, "ki": 1.0, "kd": 10.0, "d_alpha": 0.5 },
    "target": { "target": 80, "targetOffset": 3, "negHyst": 2, "posHyst": 1 },
    "limits": { "minSpeed": 30, "normalMaxSpeed_softcap": 60,
                "slew_up": 8, "slew_down": 4, "ema_n": 5 },
    "safety": { "critical_margin": 5, "fan_fault_pwm": 100 }
  }]
}
```

Measured-good starting point (Arista 7060X6 / quicksilver):
`kp=0.075, ki=1, kd=10, targetOffset=3, negHyst=2, posHyst=1`.

---

## 6. Migration

1. Land the single-algorithm `ThermalManager` + JSON loader in `sonic-platform-common`.
2. Port the Arista `CoolingLogicIncPid` guards (deadband, filtered-D, slew, soft cap,
   critical clamp, fail-safe) into it verbatim — this is the measured code.
3. Translate each SKU's tuning into the JSON above; `get_thermal_manager()` returns the
   generic manager. Keep the old logic behind a flag for one release and A/B the PWM
   trace against the measured baseline.
4. Default on per-SKU once traces match; remove the legacy path a release later.

A **firmware fail-safe floor must remain in the BMC/CPLD**: if `thermalctld` dies, fans
go to a safe speed. SONiC must never be the only line of defense.

---

## 7. Open questions

- **Warm/fast reboot**: re-seed `pwm` from the current measured fan speed and
  `T_prev/T_prev2` from the first two reads so the loop doesn't slam to 100%.
- **Tick rate**: the fastest sensor (ASIC ~3 s) should drive the loop; don't run it on
  the 60 s default. Use `get_interval()` / a sub-loop.
- **Soft cap value** is the main power knob — set `normalMaxSpeed_softcap` per SKU from
  the measured safe ceiling (49% held on quicksilver with no thermal penalty).

---

## 8. Bottom line

- One algorithm: **hardened incremental PID**. Multi-phase per tick, single code path.
- It is chosen because it is **measured**, not because it abstracts multiple vendors.
- The deadband / filtered-D / slew / soft-cap / critical-clamp / fail-safe pieces are
  in-loop guards that make the PID both safe and power-saving — they are the reason it
  beats the baseline (~-16.6 W, 82%->49% peak fan, no thermal penalty).
