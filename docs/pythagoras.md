# Pythagoras Kinematics (implementation notes)

This document describes the current behavior of
`klippy/kinematics/pythagoras.py`.

## Overview

`PythagorasKinematics` defines a 3-axis kinematics class using:

- `stepper_a` and `stepper_b` for XY motion through a custom iterative solver (`pythagoras_stepper_alloc`)
- `stepper_z` as a cartesian Z axis (`cartesian_stepper_alloc`)

At startup it:

1. Reads XY homing coordinates from `[printer]` (`home_x`, `home_y`)
2. Builds three rails (`a`, `b`, `z`)
3. Configures the A/B iterative solvers from per-stepper geometry
4. Computes A/B endstop positions from `(home_x, home_y)` and overrides rail endstops
5. Registers step generators and motion limits

## Required configuration

### `[printer]`

- `home_x` (float): XY position used as the logical XY home target
- `home_y` (float): XY position used as the logical XY home target
- `homing_retract_dist` (float, default `399`): distance for force-move retract during XY homing
- `homing_speed` (float, default `200`): speed for the homing retract move
- `homing_accel` (float, default `50`): acceleration for the homing retract move

### `[stepper_a]` and `[stepper_b]`

In addition to normal stepper options used by `LookupMultiRail`, the implementation expects:

- `pulley_x` (float)
- `pulley_y` (float)
- `pulley_r` (float)
- `tip_r` (float)
- `position_max` (float)

`pulley_x/pulley_y/pulley_r/tip_r` are passed directly to `pythagoras_stepper_alloc`.

`[stepper_a]` also accepts optional homing parameters:

- `homing_current` (float, default `0.4`): TMC current used during XY homing retract
- `homing_dwell` (float, default `0.1`): dwell time (seconds) before disabling stepper B during homing

### `[stepper_z]`

Uses standard cartesian Z behavior via `cartesian_stepper_alloc`.

### Optional kinematics-level limits

- `max_z_velocity` (defaults to toolhead max velocity)
- `max_z_accel` (defaults to toolhead max accel)

These apply when a move contains Z motion.

## Coordinate transforms

### Forward transform (XY -> A/B)

`_calc_steppers_from_xy(x, y)` calls:

- `rails[0].calc_position_from_coord((x, y, 0))`
- `rails[1].calc_position_from_coord((x, y, 0))`

and returns the corresponding A/B rail positions.

### Inverse transform (A/B -> XY)

`calc_position()` obtains A/B/Z rail positions and computes XY by numerical minimization:

- Objective: minimize squared error between target `(a, b)` and forward-computed `(a', b')`
- Solver: `scipy.optimize.minimize(..., method='BFGS')`
- Initial guess: `[10, home_y/2]`

The result is returned as `[x, y, z]`.

## Endstop initialization

During `__init__`, A/B endstops are recalculated from configured XY home:

- `endstops = _calc_steppers_from_xy(home_x, home_y)`
- `rail_a.position_endstop = endstops[0]`
- `rail_b.position_endstop = endstops[1]`

This ties A/B home endstops to a cartesian XY home target.

## Homing behavior

### Axis selection

- XY are always treated as a pair (home both if either X or Y is requested)
- Z homes independently through `_home_axis()`

### Current XY homing implementation

Current `home()` logic for XY is custom and not generic:

1. Reads `home_x/home_y`
2. Dwells for `homing_dwell` seconds, then disables motor for stepper B
3. If a TMC driver is configured for stepper A, reads the current `run_current` and temporarily sets TMC current to `homing_current`
4. Performs `force_move.manual_move(stepper_a, -homing_retract_dist, homing_speed, homing_accel)`
5. If TMC current was changed, restores it to the value read from the TMC driver
6. Sets toolhead position to `[home_x, home_y, current_z, current_e]`

Notes:

- If no TMC driver is configured for stepper A, the current-change steps are skipped

### Z homing

Z uses generic `_home_axis()` logic based on rail homing info and min/max range.

## Position setting and homed state

- `set_position(newpos, homing_axes)` updates all steppers to `newpos`
- If axis 2 (Z) is homed, `limit_z` is set to Z rail range
- `note_z_not_homed()` and motor-off event reset `limit_z` to `(1.0, -1.0)`

`get_status()` reports:

- `homed_axes`: currently always includes `xy`; includes `z` only when `limit_z` indicates homed
- `axis_minimum` and `axis_maximum` from rail ranges

## Move checking and limits

`check_move(move)` currently:

- Does not enforce XY kinematic boundary checks (TODO in code)
- For moves with Z component, scales speed/accel by Z contribution:
  - `z_ratio = move.move_d / abs(move.axes_d[2])`
  - Limits applied: `max_z_velocity * z_ratio`, `max_z_accel * z_ratio`

## Debug/experimental helpers in code

The module contains internal helper functions:

- `_test_calc_position()` for round-trip XY/A-B checks
- `_round_to_rail()` and `_round_to_nearest_full_step()` for full-step snapping experiments

These are not part of normal motion flow.

## Current limitations and TODOs

The implementation includes explicit TODOs and partial behavior:

- XY homing is custom/hard-coded, not a generalized rail homing strategy
- XY limit checking in `check_move()` is not implemented
- Z endstop checks in `check_move()` are marked TODO
- Inverse kinematics relies on generic optimization each call (BFGS)

## Entry point

`load_kinematics(toolhead, config)` returns `PythagorasKinematics(toolhead, config)`.