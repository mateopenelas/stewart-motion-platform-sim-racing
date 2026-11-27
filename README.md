# 6-DOF Stewart Motion Platform for Sim Racing

Motion cueing software for a DIY 6-DOF Gough–Stewart motion platform, driven by live telemetry from Assetto Corsa. The core is a Python implementation of the classical washout algorithm from Reid & Nahon (1985).

> **Status:** the software runs on live Assetto Corsa telemetry and drives a 3D visualization of the platform. No hardware has been assembled yet: the parts for the first linear actuator have been purchased. Inverse kinematics (platform pose → leg lengths) is not implemented yet.

## How it works

```
Assetto Corsa shared memory
  → telemetry reader (ctypes + mmap)
  → axis conversion (AC frame → platform frame)
  → scaling and limiting
  → classical washout: translational channel, tilt coordination, rotational channel
  → platform pose (x, y, z, roll, pitch, yaw)
  → 3D visualization (matplotlib)
```

The physics/washout loop and the matplotlib render run in separate threads and share the platform pose under a lock. With the 3D window open, rendering still takes time away from the physics loop (48.6% of telemetry packets missed at 333 Hz); with `solo_fisica = True` the loop reads all of them.

## Implementation notes

- **Source:** Chapter 5 of Reid & Nahon (1985). Final orientation is β_S = β_SL + β_SH (tilt coordination + rotational channel).
- **Translational channel:** a 2nd-order and a 1st-order high-pass filter in series, giving the 3rd order needed for the platform position to return to neutral after a sustained acceleration (Eq. 2.26).
- **Tilt coordination:** uses the exact Eq. 5.30 (`arctan2`) instead of the linearized Eq. 5.34, because a car's vertical specific force is not constant (aerodynamic load, kerbs, compressions). The tilt rate is limited to 5.8°/s and the angle to 20°. Both limits are applied to the tilt state, not to an output copy, to avoid windup.
- **Warm start:** filter states are initialized on the first frame, so the output starts at exactly zero.
- **Erratum in Eq. 5.22:** the third term is missing a `tan(θ_S)` factor. The correction was found handwritten on the scanned copy of the paper.

## Assetto Corsa shared-memory findings

These are not documented by Assetto Corsa. They were determined experimentally:

- `localAngularVelocity` indices are `[0] = pitch`, `[1] = yaw`, `[2] = roll` (not the same order as `accG`). Pitch was isolated on the Suspension Test Track, yaw by correlating with `steerAngle`, roll by elimination.
- `accG` is kinematic acceleration, not specific force, expressed in G.
- The coordinate system is right-handed.

## Known limitations

- **Tuning** (`K_tras = 0.15`, translational high-pass ω = 0.5) was chosen only to keep excursions within a reasonable range. It has not been validated on hardware or against human perception.
- **Tilt rate limit:** at 5.8°/s, reaching 20° takes about 3.4 s, longer than a typical braking event. The tilt can still be moving when the event is over, which produces false cues in quick direction changes. This is one of the motivations for moving to optimal (LQR) washout.
- **Platform geometry** (base/platform radii 350/250 mm, 10° anchor spacing) uses placeholder values.
- **Symmetric limits:** the scaling and limiting are symmetric, while a car brakes much harder than it accelerates.

## Roadmap

- Assembly and bring-up of the first actuator (ball screw + 80ST servo, based on the Departed Reality PS500 design with generic parts)
- Inverse kinematics (pose → six leg lengths)
- Optimal (LQR) washout, Reid & Nahon Chapter 6
- Vestibular models

## Running it

Requires Windows and Assetto Corsa running with an active session (the shared memory only exists while a session is loaded).

```
pip install -r requirements.txt
cd src
python Simulacion.py
```

Set `solo_fisica = True` in `Simulacion.py` to run the washout without the 3D window (used for timing measurements).

## Project log

[`docs/bitacora.txt`](docs/bitacora.txt) is the engineering log kept during development (in Spanish): decisions, bugs, experiments and open questions.

## Reference

Reid, L. D., & Nahon, M. A. (1985). *Flight Simulation Motion-Base Drive Algorithms: Part 1 – Developing and Testing the Equations*. UTIAS Report No. 296, University of Toronto Institute for Aerospace Studies.

## License

MIT, see [LICENSE](LICENSE).
