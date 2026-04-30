# Joint Movement Flow

This is an outline of the code paths and data structures that can result in
joint movement on the `multi-robot-architecture` branch.

## 1. Runtime And Coordinate Owners

- `src/gradient_os/run_controller.py`
  - Loads the desired/effective runtime config.
  - Selects the active `RobotConfig`.
  - Calls `robot_config.set_active_robot(...)`.
  - Creates and registers the active actuator backend.
  - Owns the UDP controller command loop.
- `src/gradient_os/arm_controller/robots/base.py`
  - `RobotConfig` is the robot policy contract.
  - Key movement fields:
    - `num_logical_joints`
    - `num_physical_actuators`
    - `actuator_ids`
    - `logical_to_physical_map`
    - `logical_joint_limits_rad`
    - `actuator_encoder_counts_per_rev`
    - `actuator_gear_ratios`
    - `actuator_position_signs`
    - `actuator_counts_per_radian`
    - `default_servo_backend`
    - `default_ik_solver_backend`
- `src/gradient_os/arm_controller/robot_config.py`
  - Backward-compatible module-level mirror of the active `RobotConfig`.
  - Existing controller/planner modules still read values like
  `NUM_LOGICAL_JOINTS`, `SERVO_IDS`, `LOGICAL_TO_PHYSICAL_MAP`,
  `LOGICAL_JOINT_LIMITS_RAD`, and default profile values from here.
- `src/gradient_os/arm_controller/utils.py`
  - `current_logical_joint_angles_rad` stores the latest known/logical command
  or feedback vector.
  - `trajectory_state` is the shared controller motion state:
  `is_running`, `should_stop`, jog state, active program metadata, RTCore
  settle IDs, diagnostics flags, and stop latches.

## 2. Shared Movement Data Shapes

- Logical joint vector
  - Python shape: `list[float]`.
  - Units: radians.
  - Length: active robot `num_logical_joints`.
  - Used by IK, bounded joint moves, trajectory execution, `servo_driver`, and
  backend interfaces.
- Joint path
  - Python shape: `list[list[float]]`.
  - Each inner list is one logical joint vector in radians.
  - Paired with an execution frequency in Hz.
  - Produced by bounded joint interpolation, IK path planning, trajectory
  program planning, and preview/weld planners.
- Planned trajectory step
  - Python shape: `dict`.
  - Common forms:
    - `{"type": "move", "path": joint_path, "freq": hz, ...}`
    - `{"type": "pause", "duration": seconds}`
    - Legacy: `{"type": "joint_move", "target_q": ..., "speed": ..., ...}`
  - Consumed by `_trajectory_executor_thread(...)`.
- API-to-controller command payload
  - Most structured controller commands are UDP strings with optional base64
  JSON payloads.
  - Examples:
    - `APPLY_JOINT_SETPOINT,<b64-json>`
    - `APPLY_JOINT_DELTA,<b64-json>`
    - `JOG_SESSION_START,<b64-json>`
    - `JOG_SESSION_UPDATE,<b64-json>`
  - Simpler commands use comma tokens, for example `RUN_TRAJECTORY,name,...`,
  `MOVE_LINE,...`, `ROTATE,...`, and `SET_ORIENTATION,...`.
- RTCore trajectory point
  - Python-side dict before serialization:
    - `positions_rad`: logical joint vector
    - `axis_q`: optional RTCore axis-space position vector
    - `qd`: optional RTCore axis-space velocity vector
    - `axis_mask`
    - `flags`
    - `t_from_start_ns`
  - Serialized by `_TRAJECTORY_POINT_STRUCT` in the EtherCAT RTCore backend.
- RTCore jog command
  - Python-side input: logical joint velocity vector in rad/s.
  - Converted to RTCore `axis_qd`.
  - Sent through the RTCore command ring as an active jog command with a
  motor-side watchdog timeout.

## 3. UI And API Entry Points

- Direct joint step
  - `web-ui/src/ControlPanel.tsx`
    - `handleJointStep(...)`
    - Posts `/control/joint-jog` with:
      - `joint`
      - `delta_deg`
      - `wait_for_idle`
  - `src/gradient_os/api/main.py`
    - `/control/joint-jog`
    - Encodes `APPLY_JOINT_DELTA,<b64-json>` for the controller.
- Home and rest poses
  - `web-ui/src/App.tsx` and `web-ui/src/ControlPanel.tsx`
    - Post `/control/home` or `/control/rest`.
  - `src/gradient_os/api/main.py`
    - `/control/home` sends `APPLY_JOINT_SETPOINT` with all-zero
    `arm_angles_rad`, `max_motor_rpm`, and `canonical_wrap_target=true`.
    - `/control/rest` sends `APPLY_JOINT_SETPOINT` with the resolved rest pose.
- Cartesian and orientation commands
  - `web-ui/src/ControlPanel.tsx`
    - Posts `/control/move-line-relative`.
    - Posts `/control/rotate`.
  - `src/gradient_os/api/main.py`
    - Forwards `MOVE_LINE_RELATIVE`, `ROTATE`, and `SET_ORIENTATION` UDP
    commands.
- Trajectory and weld programs
  - `web-ui/src/App.tsx`
    - Plans previews and welds through `/trajectory/plan-points` and
    `/trajectory/plan-weld`.
    - Runs saved or preview trajectories through `/trajectory/run`.
  - `src/gradient_os/api/main.py`
    - Preview planning executes in the API process after syncing local planner
    runtime to the controller runtime.
    - `/trajectory/run` forwards `RUN_TRAJECTORY,name,use_cache,loop_override`.
- Controller-owned jog sessions
  - `web-ui/src/ControlPanel.tsx`
    - Builds a six-value Cartesian jog vector:
      - `vx`, `vy`, `vz` in m/s
      - `v_roll`, `v_pitch`, `v_yaw` in deg/s
    - Posts `/control/jog/session/start`, `/update`, and `/stop`.
    - Maintains sequence numbers, owner ID, deadman state, lease timeout, and a
    keepalive timer.
  - `src/gradient_os/api/main.py`
    - Encodes the session payloads into `JOG_SESSION_*` controller commands.

## 4. Controller Dispatch

- `src/gradient_os/run_controller.py`
  - Receives UDP command text.
  - Parses comma tokens and base64 JSON payloads.
  - Calls `src/gradient_os/arm_controller/command_api.py`.
  - Sends plain ACK/ERROR replies or structured base64 JSON ACK/ERROR payloads.
- Main movement command dispatch:
  - `APPLY_JOINT_DELTA` -> `command_api.handle_apply_joint_delta(...)`
  - `APPLY_JOINT_SETPOINT` -> `command_api.handle_apply_joint_setpoint(...)`
  - `JOG_SESSION_*` -> jog session handlers in `command_api.py`
  - `MOVE_LINE`, `MOVE_LINE_RELATIVE` -> profiled Cartesian move handlers
  - `ROTATE`, `SET_ORIENTATION` -> orientation path handlers
  - `RUN_TRAJECTORY` -> `command_api.handle_run_trajectory(...)`
  - fallback raw joint-angle command -> `servo_driver.set_servo_positions(...)`

## 5. Direct Joint Setpoint And Delta Flow

- Delta path
  - `command_api.handle_apply_joint_delta(...)`
    - Validates the 1-based joint number.
    - Converts to a 0-based target joint index.
    - Requires target axes to be motion-ready.
    - Reads a baseline from `_bounded_joint_baseline(...)`.
    - Builds `target_q` by applying `delta_deg`.
    - Delegates to `handle_apply_joint_setpoint(...)`.
- Setpoint path
  - `command_api.handle_apply_joint_setpoint(...)`
    - Coerces speed and acceleration.
    - If `max_motor_rpm` is present:
      - Checks motion readiness.
      - Reads current joint state.
      - Builds a bounded joint path with `_build_bounded_joint_path(...)`.
      - Starts `trajectory_execution._open_loop_executor_thread(...)`.
      - Returns accepted motion metadata.
    - If `max_motor_rpm` is not present:
      - Calls `servo_driver.set_servo_positions(...)`.
      - This becomes a backend setpoint or legacy sync write.
- Bounded joint path
  - `_build_bounded_joint_path(...)`
    - Computes joint deltas.
    - Uses robot gear ratios and `max_motor_rpm`, or a max joint speed.
    - Creates a smoothstep-interpolated `joint_path`.
    - Runs at `_SAFE_JOINT_MOVE_FREQUENCY_HZ`.

## 6. Cartesian, Orientation, And Program Planning Flow

- Single Cartesian/orientation commands
  - `command_api.handle_move_profiled(...)`
    - Reads current control joint feedback.
    - Plans a Cartesian path into a `joint_path`.
    - Chooses open-loop or closed-loop executor.
  - `command_api.handle_rotate_command(...)`
    - Reads current control pose.
    - Builds a target orientation.
    - Calls `_execute_orientation_path(...)`.
  - `command_api.handle_set_orientation_command(...)`
    - Builds an absolute orientation target.
    - Calls `_execute_orientation_path(...)`.
  - `_execute_orientation_path(...)`
    - Builds SLERP orientation samples.
    - Calls `ik_solver.solve_ik_path_batch(...)`.
    - Executes the resulting `joint_path`.
- Saved trajectory program run
  - `command_api.handle_run_trajectory(...)`
    - Loads a recorded trajectory JSON.
    - Reads the current/best joint state.
    - Converts authored moves into `planned_steps`.
    - `home` and authored `move` become bounded joint paths via
    `_plan_joint_move_to_pose(...)`.
    - Cartesian `move_relative`, `move_absolute`, and `move_arc` become dense
    IK-derived joint paths.
    - Optionally collapses compatible move/pause steps into one compound path.
    - Starts `_trajectory_executor_thread(...)` or loop-specific executor.
- Program execution
  - `trajectory_execution._trajectory_executor_thread(...)`
    - Iterates `planned_steps`.
    - For `type == "move"`, calls `_execute_joint_path(...)`.
    - For `type == "pause"`, holds time without commanding new points.
  - `_execute_joint_path(...)`
    - Calls `_open_loop_executor_thread(...)` in the current program thread.

## 7. Jog Flow

- Session state
  - `src/gradient_os/arm_controller/jog_session.py`
    - `JogSessionRecord` stores:
      - `session_id`
      - `owner_id`
      - `state`
      - `lease_timeout_s`
      - `lease_deadline_monotonic`
      - `deadman`
      - `velocity_vector`
      - `gripper_velocity_deg_s`
      - `last_seq_received`
      - `last_seq_applied`
      - `backend_mode`
      - commanded pose and joint snapshots
      - gate failure/following error metadata
    - `JogSessionManager` owns start/update/stop, lease expiry, stale sequence
    rejection, and snapshots.
- Jog session start/update
  - `command_api.handle_jog_session_start(...)`
    - Rejects if another motion is running.
    - Verifies control feedback at arm time.
    - Selects execution policy:
      - `joint_velocity_lease` when the active backend supports RTCore jog.
      - `controller_cartesian_loop` otherwise.
    - Starts the jog controller thread.
  - `handle_jog_session_update(...)`
    - Updates lease, deadman, velocity vector, and sequence number.
- Jog controller loop
  - `command_api._jog_controller_thread(...)`
    - Runs at `JOG_CONTROL_FREQUENCY_HZ`.
    - Reads current control joint feedback.
    - Computes current FK pose.
    - Integrates the UI Cartesian velocity command over `dt`.
    - Solves for a next joint command through Jacobian DLS when available, or
    IK fallback.
    - Validates joint step, limits, residuals, and command drift.
    - If accepted:
      - RTCore path sends `update_joint_velocity_lease_jog(...)` with rad/s
      joint velocities.
      - fallback path sends `servo_driver.set_servo_positions(...)`.
      - Stores accepted commanded pose/joint vector back into
      `JogSessionManager`.
    - If rejected:
      - Sends zero jog velocity or holds the backend jog lease.
      - Records gate failure telemetry.

## 8. Executor And Backend Convergence

- `src/gradient_os/arm_controller/trajectory_execution.py`
  - `_open_loop_executor_thread(...)`
    - Primary convergence point for scheduled joint paths.
    - If the active backend exposes `submit_joint_trajectory(...)`, the full
    path is offloaded to RTCore.
    - Otherwise, it precomputes sync-write commands and emits them from Python.
  - `_closed_loop_executor_thread(...)`
    - Reads feedback each cycle.
    - Builds corrected command tuples.
    - Writes through backend `sync_write(...)` or legacy `servo_protocol`.
- `src/gradient_os/arm_controller/servo_driver.py`
  - Compatibility layer for older call sites.
  - `set_servo_positions(...)`
    - Updates `utils.current_logical_joint_angles_rad`.
    - Uses `backend.set_joint_positions(...)` when an initialized backend is
    present.
    - Falls back to legacy physical-servo sync-write conversion.
  - `get_current_arm_state_rad(...)`
    - Strict read path.
  - `get_control_arm_state_rad(...)`
    - Motion-control read path.
    - Uses backend `get_control_joint_positions(...)` when available.
- `src/gradient_os/arm_controller/actuator_interface.py`
  - Backend contract for movement:
    - `set_joint_positions(...)`
    - `prepare_sync_write_commands(...)`
    - `sync_write(...)`
    - `sync_read_positions(...)`
    - `raw_to_joint_positions(...)`
    - optional leased jog methods:
      - `supports_joint_velocity_lease_jog(...)`
      - `start_joint_velocity_lease_jog(...)`
      - `update_joint_velocity_lease_jog(...)`
      - `stop_joint_velocity_lease_jog(...)`

## 9. EtherCAT RTCore Handoff

- `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`
  - Active production path for `Gradient-05`.
  - Key runtime structs:
    - `_ShmHeader`
    - `_AxisConfig`
    - `RTCoreExecutionStatus`
    - `RTCoreTrajectorySubmission`
    - `RTCoreJogDebugStatus`
- Scheduled trajectory handoff
  - `submit_joint_trajectory(...)`
    - Resolves requested frequency to RTCore cycle timing.
    - Estimates joint velocities.
    - Calls `begin_trajectory(...)`.
    - Builds per-point dicts.
    - Calls `enqueue_trajectory_points(...)`.
    - Calls `commit_trajectory(...)`.
  - `enqueue_trajectory_points(...)`
    - Converts `positions_rad` to RTCore `axis_q`.
    - Converts joint velocities to RTCore `axis_qd`.
    - Aligns continuous 607A command targets to the selected turn reference.
    - Enforces first-point and per-step wire-frame safety gates.
    - Serializes each point into `_TRAJECTORY_POINT_STRUCT`.
  - RTCore consumes the command ring and drives the EtherCAT target-position
  stream.
- One-point setpoint handoff
  - `EthercatRTCoreBackend.set_joint_positions(...)`
    - Wraps a direct setpoint as a one-point RTCore trajectory.
    - Commits it through the same trajectory command ring.
- Leased jog handoff
  - `update_joint_velocity_lease_jog(...)`
    - Calls `send_realtime_jog_command(...)`.
    - Converts logical joint velocities to `axis_qd`.
    - Sends an active jog command with `axis_mask`, flags, and timeout.
  - `stop_joint_velocity_lease_jog(...)`
    - Sends a stop/optional quick-stop jog command.

## 10. Safety And Stop Gates

- Motion exclusivity
  - `utils.trajectory_state["is_running"]` blocks competing scheduled moves.
  - Jog session start rejects while scheduled motion is running.
  - `handle_run_trajectory(...)` stops active jog before program execution.
- Feedback readiness
  - Direct/jog/planned paths generally use `servo_driver.get_control_arm_state_rad(...)`.
  - Strict canonical reads still exist through `get_current_arm_state_rad(...)`
  and diagnostics.
- Drive and target readiness
  - `_require_target_axes_motion_ready(...)` gates direct bounded moves.
  - RTCore backend hard-fails control feedback on unsafe DS402/fault states.
- Wire-frame safety
  - RTCore trajectory upload checks:
    - first-point deviation from live/hold reference
    - per-point oversized joint-space step
    - seam-crossing restrictions where enabled
    - continuous 607A turn alignment
- Stop
  - `/control/stop` -> `STOP` -> `command_api.handle_stop_command(...)`.
  - Sets stop latch and `should_stop`.
  - Stops active jog.
  - Aborts RTCore trajectory if supported.
  - Skips legacy brake writes for RTCore-backed stop semantics.

## 11. Debugging Checklist

- Identify the entry point:
  - `/control/joint-jog`
  - `/control/home` or `/control/rest`
  - `/control/move-line-relative`, `/control/rotate`, `/control/set-orientation`
  - `/trajectory/run`
  - `/control/jog/session/*`
- Identify the data shape in flight:
  - direct target: `arm_angles_rad`
  - delta target: `joint` + `delta_deg`
  - planned path: `joint_path`
  - program: `planned_steps`
  - jog: `velocity_vector` -> `q_dot_rad_s`
  - RTCore: `positions_rad` -> `axis_q`, `qd` -> `axis_qd`
- Confirm the convergence point:
  - scheduled path: `_open_loop_executor_thread(...)`
  - RTCore trajectory: `submit_joint_trajectory(...)`
  - RTCore jog: `update_joint_velocity_lease_jog(...)`
  - legacy/direct fallback: `servo_driver.set_servo_positions(...)`
- Confirm completion semantics:
  - some endpoints return controller acceptance only.
  - bounded joint jogs may request `wait_for_idle`.
  - trajectory programs report controller program-thread acceptance, not always
  immediate physical completion.
  - RTCore status is authoritative for queued trajectory and jog execution.