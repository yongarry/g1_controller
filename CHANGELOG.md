# Changelog

All notable changes to this repository are documented here.

## 2026-09-04

### Added

- `cmd/gen_shatters_obstacles.py`: joystick-mode scatter of box shards on a flat
  10×10 ground. Per-axis full-size ranges (`--x` / `--y` / `--z`), spawn keep-out
  (`--start-clear`, default 0.5×0.5 m). No footstep CSV.
- `cmd/gen_cmd.py --realistic`: after a large `step_x` / `step_y` / `|step_z|`,
  the next sample on that axis is drawn from the non-extreme band so consecutive
  extremes cannot stack landing error past the trained command limits.

### Changed

- `cmd/gen_rocky_mountain.py`: `--flush-drop` (default 0.2 m) applies only on
  each stepping stone's local −x (approach) side so the swing foot is not snagged
  while climbing; local +x stays flush with the stone top.
- Scene generators strip each other's marked XML regions, including shatter
  obstacles, so footstep / rocky / ArUco / shatter blocks never stack.
- Default 3-D command sampling in `gen_cmd.py`:
  `x ∈ [0.2, 0.4]`, `y ∈ [0.2, 0.4]`, `z ∈ [-0.15, 0.2]`, `yaw ∈ [-0.4, 0.4]`.
- Footstep `deploy_base.yaml` defaults: `ssp_t` 0.6 s, swing `height` 0.07 m,
  `ik_iters` 1, `foot_state_source: sim_odom`.
- `obs_wocomz_heuri` deploy uses `260703_1430_3d_prev.onnx` with
  `com_generate_type: prev`.
