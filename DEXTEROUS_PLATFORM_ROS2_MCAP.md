# Franka + LinkerHand + RealSense Dexterous Platform

This document defines the long-term data collection and deployment architecture for
the FR3 + LinkerHand dexterous manipulation stack. LeRobot remains useful as a
training/export format, but it should not be the only raw data format for this
hardware.

## Why LeRobot Alone Is Not Enough

The current LeRobot record loop is intentionally simple:

1. Read one robot observation.
2. Read teleop or policy action.
3. Send action.
4. Save one synchronized dataset frame.

That model is good for arms with a small gripper and RGB cameras. For this
platform, it hides the timing information that matters most:

- Franka publishes high-rate arm state and accepts commands with controller
  timing.
- LinkerHand has joint state, command state, and potentially pressure/tactile
  streams with separate update rates.
- RealSense RGB-D frames have device timestamps, frame numbers, USB arrival
  times, and occasional latency spikes.
- Contact-rich manipulation depends on the exact ordering of hand closure,
  tactile onset, arm motion, and image evidence.

Therefore, the raw dataset format should be asynchronous ROS2 topics recorded
to MCAP. LeRobot, OpenPI, DROID, and robomimic should be generated views of that
raw record.

## Recommended Stack

```text
ROS2 main system
  |
  |-- franka_ros2 / libfranka
  |-- linkerhand_ros2_sdk
  |-- realsense-ros
  |-- teleop input nodes
  |     |-- mocap retargeting
  |     |-- VR / SpaceMouse / keyboard
  |     `-- policy action server
  |-- recorder
  |     |-- rosbag2 storage: MCAP
  |     |-- episode/event markers
  |     `-- timestamp audit sidecar
  |-- converters
  |     |-- MCAP -> LeRobotDataset
  |     |-- MCAP -> OpenPI
  |     |-- MCAP -> robomimic HDF5
  |     `-- MCAP -> DROID-style layout
  `-- deployment
        |-- policy runtime node
        |-- action safety/filter node
        `-- controller command publishers
```

## Current Repository Fit

The existing code already has useful pieces:

- `lerobot/src/lerobot/robots/fr3_eef/` is already a ROS2-backed LeRobot robot.
  It publishes FR3 end-effector pose commands and LinkerHand joint commands,
  subscribes current pose and hand state, and exposes a LeRobot feature schema.
- `lerobot/src/lerobot/teleoperators/mocap_retarget/` already retargets mocap
  hand data through dex-retargeting and emits the same FR3 EE + LinkerHand
  action schema.
- `lerobot/src/lerobot/scripts/lerobot_record.py` can remain the quick path for
  small experiments, but its frame timestamp is computed as `frame_index / fps`,
  not from the hardware clocks.

The new system should reuse the schema decisions from these modules, but move
raw logging outside the LeRobot synchronous record loop.

## Topic Contract

Use explicit topic groups and record both command-side and measured-side data.

### Franka

Required:

- `/franka/joint_states`: measured joint position, velocity, effort.
- `/franka/robot_state`: robot mode, error flags, external wrench if available.
- `/franka/ee_pose`: measured end-effector pose in the base frame.
- `/franka/ee_twist`: measured end-effector twist if available.
- `/franka/command/ee_pose`: commanded Cartesian target.
- `/franka/command/joint`: commanded joint target if joint policies are used.

Recommended metadata:

- controller name and version
- impedance/stiffness/damping parameters
- collision thresholds
- command sequence number
- publish time and intended execution time

### LinkerHand

Required:

- `/linkerhand/right/joint_states`: measured joint positions.
- `/linkerhand/right/command`: command sent to the hand.
- `/linkerhand/right/status`: device status, error code, temperature if exposed.

If the hand supports tactile or pressure:

- `/linkerhand/right/tactile`: raw pressure/tactile array.
- `/linkerhand/right/contact_events`: derived contact onset/release events.

For long-term stability, prefer radian joint topics for model training and keep
0-255 actuator command topics as hardware-specific command logs.

### RealSense

Required per camera:

- `/camera/<name>/color/image_rect_raw`
- `/camera/<name>/color/camera_info`
- `/camera/<name>/depth/image_rect_raw`
- `/camera/<name>/depth/camera_info`
- `/camera/<name>/metadata`

The metadata message or sidecar must include:

- device timestamp
- ROS publish timestamp
- recorder receive timestamp
- frame number
- stream type
- exposure/gain when available

### Teleop and Policy

Required:

- `/teleop/raw`: raw human input or mocap skeleton summary.
- `/teleop/retargeted_action`: action after retargeting.
- `/policy/observation_query`: synchronized observation sent to the model.
- `/policy/action`: raw policy output.
- `/policy/action_filtered`: action after safety limits, interpolation, clipping.
- `/policy/inference_trace`: latency, model id, pre/post-process timing.

## Episode and Event Markers

Record episodes as events, not only as file boundaries:

- `/episode/event`: `start`, `success`, `failure`, `reset`, `discard`.
- `/episode/task`: natural-language task and structured task id.
- `/episode/operator`: operator id or anonymized session id.
- `/episode/environment`: object ids, scene setup, calibration version.
- `/episode/contact_phase`: optional labels such as approach, first contact,
  grasp, lift, place, release.

This makes it possible to re-cut episodes later without losing raw data.

## Timestamp Rules

Every message should carry these concepts when possible:

- `source_time`: time from the device or controller.
- `publish_time`: ROS header stamp at the publishing node.
- `receive_time`: recorder wall-clock time when the message was received.
- `sequence_id`: monotonic source sequence number.

Use the same machine clock for ROS nodes whenever possible. If multiple
machines are used, require chrony or PTP and record clock offset diagnostics.

Do not overwrite source timestamps during conversion. Generated training frames
should contain both the resampled training timestamp and the original source
timestamps for selected modalities.

## Timestamp Audit

The recorder should write an audit sidecar per MCAP file:

```json
{
  "recording_id": "2026-06-12_fr3_linkerhand_redcube_001",
  "topics": {
    "/camera/handeye/color/image_rect_raw": {
      "count": 1800,
      "nominal_hz": 30,
      "median_period_ms": 33.3,
      "p99_period_ms": 48.0,
      "dropped_sequence_count": 2,
      "median_receive_latency_ms": 18.4,
      "p99_receive_latency_ms": 61.2
    }
  },
  "sync_windows": {
    "policy_30hz": {
      "max_camera_age_ms": 80,
      "max_arm_state_age_ms": 20,
      "max_hand_state_age_ms": 30,
      "max_tactile_age_ms": 20
    }
  }
}
```

Minimum pass/fail checks:

- No missing required topics.
- No camera frame-number jumps above configured tolerance.
- P99 camera receive latency below task-specific limit.
- Arm state age at each policy query below controller limit.
- Hand state and tactile age at each policy query below contact-phase limit.
- Command-to-state lag estimated and stored.

## MCAP to Training Dataset Conversion

The converter should be deterministic and config-driven.

Input:

- MCAP file or directory.
- episode event topic.
- output format: LeRobot, OpenPI, robomimic, or DROID.
- target sampling clock, usually policy/action time.
- per-modality lookup rule: nearest, previous, interpolation, or window.

Recommended default for LeRobot export:

```text
training frame timestamp = policy/action command timestamp
arm state                  = nearest or interpolated around timestamp
hand state                 = nearest or previous-valid sample
tactile                    = short history window before timestamp
camera RGB-D               = latest frame with device_time <= timestamp
action                     = command intended for timestamp + action_horizon
```

The exported LeRobot dataset should include standard observation/action fields
compatible with the current `fr3_eef` schema, plus optional timing fields:

- `observation.timing.arm_state_age_ms`
- `observation.timing.hand_state_age_ms`
- `observation.timing.camera_<name>_age_ms`
- `observation.timing.tactile_age_ms`
- `action.timing.command_to_state_lag_ms`

If a downstream format cannot store these fields cleanly, keep them in a
sidecar parquet/jsonl file keyed by episode and frame index.

## Deployment Architecture

Do not deploy policies by calling LeRobot record/eval loops directly for the
final robot stack. Use ROS2 runtime nodes:

```text
observation synchronizer
  -> model preprocessor
  -> policy runtime
  -> action postprocessor
  -> safety/filter/interpolator
  -> command publishers
```

Runtime requirements:

- Fixed policy tick, for example 10, 20, or 30 Hz.
- Higher-rate command interpolation for Franka if needed.
- Strict observation age limits. If any required modality is stale, hold,
  slow down, or retreat depending on the safety state.
- Action limits in Cartesian pose, joint velocity, hand joint velocity, and
  tactile/contact force where available.
- Explicit model metadata in `/policy/inference_trace`: checkpoint path, git
  commit, dataset id, normalization stats hash, device, inference time.
- Watchdog that stops publishing commands if policy runtime stalls.

For early experiments, `lerobot_record.py` and `infer_local.sh` are still useful.
For contact-rich deployment, the ROS2 policy runtime should be the authority.

### Remote Policy Server

The platform can support a remote policy server/client setup, matching the
existing `scripts/server.sh` and `scripts/client.sh` pattern:

```text
GPU/server computer
  lerobot.async_inference.policy_server

robot/client computer
  cameras + Franka + LinkerHand
  observation synchronizer
  async policy client
  local safety/filter/interpolator
  ROS2 command publishers
```

In that mode the client sends synchronized images and low-dimensional robot
state to the server, receives an action or action chunk, and applies safety
locally before publishing commands. This is acceptable if the client records:

- `/policy/client_observation`: observation timestamp bundle sent to the server.
- `/policy/server_request`: request id, send time, encoded payload metadata.
- `/policy/server_response`: receive time, action sequence id, server time.
- `/policy/network_trace`: round-trip latency and dropped/late responses.
- `/policy/action_filtered`: the action that actually reaches the robot.

The server may run on a different computer, but the robot should not trust server
timing blindly. The robot-side client is the timing authority for safety.

## Implementation Phases

### Phase 1: Raw Recording Baseline

- Launch Franka, LinkerHand, RealSense, and mocap nodes under ROS2.
- Record all required topics with `rosbag2` using MCAP storage.
- Add `/episode/event` and `/episode/task` marker publisher.
- Generate timestamp audit sidecar after each recording.

Initial repository implementation:

- `dexterous_platform_ros/launch/record_fr3_linkerhand.launch.py` starts helper
  nodes for episode markers, canonical topic bridging, and recording manifest.
- `scripts/dex_record_fr3_mcap.sh` starts helper nodes and executes
  `ros2 bag record --storage mcap`.
- `scripts/dex_start_rosbag2.py` can print or execute the rosbag2 command.
- `scripts/dex_audit_samples.py` and
  `scripts/dex_convert_samples_to_lerobot_preview.py` exercise the same audit
  and conversion logic on `TopicSample` JSONL exports.

### Phase 2: LeRobot Export

- Implement `mcap_to_lerobot` converter.
- Match current `fr3_eef` observation/action schema.
- Preserve timing diagnostics as extra features or sidecars.
- Validate exported datasets against the existing LeRobot training scripts.

### Phase 3: Contact and Tactile

- Add LinkerHand tactile/pressure topics.
- Add contact onset detection and phase labels.
- Add lag estimation between hand command, measured hand state, tactile onset,
  and image evidence.

### Phase 4: Standard Policy Runtime

- Implement ROS2 policy runtime node.
- Add observation synchronizer and watchdog.
- Support LeRobot/OpenPI/robomimic model adapters through a common action
  interface.
- Log deployment episodes to MCAP with the same recorder.

### Phase 5: Multi-format Dataset Factory

- Add converters for OpenPI, robomimic HDF5, and DROID-style layout.
- Add regression tests with small MCAP fixtures.
- Add conversion manifest hashes so generated datasets can always be traced
  back to raw MCAP files and converter configs.

## Practical Rule

Use LeRobot as the convenient training format. Use MCAP as the ground truth.
When a behavior fails on the real robot, the raw MCAP should be sufficient to
answer: what did the human/model command, what did the arm do, what did the hand
do, what did the sensors see, and how old was every piece of evidence?
