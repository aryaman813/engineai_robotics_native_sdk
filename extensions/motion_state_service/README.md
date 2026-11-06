# Motion-state ROS2 service

This standalone package adds `/motion/change_state`, accepting **one requested
state per call**. All new source and build files live in this folder. No existing
SDK files, configuration, robot services, or `core/` files are changed by building
or starting this package.

The client uses a service. Internally, the server sends the existing SDK request
topic, preserving the already-tested input adapter and stop arbitration:

```text
Service client -> /motion/change_state
  -> validate actual current state and requested step
  -> /motion/state_request -> existing SDK adapter -> existing FSM
  <- actual LCM task_state feedback confirms the outcome
  -> service response
```

The existing modified SDK must still run. This is a service facade, not an
in-process replacement for the SDK subscriber. Removing the SDK topic entirely
would require changing the SDK, which is outside this folder-only change.

## Transition policy

Ordinary calls permit adjacent steps only:

```text
idle <-> pd_stand <-> walk <-> dance
```

- `passive` is valid from **any** state, including unknown/stale state.
- Recovery from passive permits `idle` or `pd_stand`, matching the SDK.
- Every ordinary step must **also** appear in the actual source state's
  `manual_transition` list in the supplied SDK motion YAML.
- In the current SDK, idle only permits passive, and PD Stand does not permit
  idle directly. The service therefore rejects direct idle/PD Stand steps despite
  their adjacency. Use passive as the intermediate state. Idle automatically
  returns to passive; dance can automatically return to walk.
- PD Stand -> dance and dance -> PD Stand are rejected by this service even
  though the SDK permits them. Go through walk, issuing separate calls and
  checking each response. The service never executes intermediate motions for you.
- An already-current ordinary target is a valid no-op; no command is sent.
- Another ordinary call is rejected while a transition is pending. Passive can
  preempt a pending call; the earlier call reports that it was preempted.

The service uses actual `task_state` LCM broadcasts from the existing
`VirtualGamepadInputAdapter`. That adapter broadcasts state even with virtual
gamepad sending disabled. The current SDK ROS2 bridge does not publish motion
state feedback, so the server listens to LCM directly. Do not run this server
against an unrelated SDK instance on the same LCM URL.

If feedback is missing or stale, ordinary requests fail without publication.
Passive can still be sent, but confirmation may time out. Dispatch always
requires a discovered SDK topic subscriber. Requests that time out are not
automatically retried or rolled back; inspect the actual FSM state before retrying.

These policy checks apply to **this service only**. The existing topic, physical
gamepad, and virtual gamepad remain available and can change state independently.
The existing FSM is the final authority. This service cannot impose an exclusive
global policy without modifying those other paths. A state can change between
validation, dispatch, and observation.

## Interface

Service type: `engineai_motion_service/srv/ChangeMotionState`

```text
string target_motion_name
---
bool valid
bool command_sent
bool state_changed
string current_motion_name
string message
```

| Field | Meaning |
| --- | --- |
| `valid` | Service-side validation passed; not an SDK acknowledgement |
| `command_sent` | A request was published through the existing topic |
| `state_changed` | Target appeared in feedback received after dispatch |
| `current_motion_name` | Latest fresh observed state, or `unknown` |
| `message` | Validation failure, no-op, preemption, confirmed outcome, or timeout |

For an already-current state, `valid=true`, `command_sent=false`, and
`state_changed=false` are expected. A confirmed state may subsequently change
automatically or through another input; confirmation does not promise it remains
active. LCM observations are not correlated SDK command acknowledgements.

## Build in the EngineAI container

First build the existing SDK and generated `interface_protocol` as described in
the [topic feature guide](../../docs/motion_state_request/README.md). Then enter
`engineai_robotics_env` and run from the repository root:

```bash
source /opt/ros/humble/setup.bash
source build/x86_64/ros2_env/install/local_setup.bash
cd extensions/motion_state_service
colcon build --base-paths src --cmake-args -DBUILD_TESTING=ON
source install/local_setup.bash
ros2 interface show engineai_motion_service/srv/ChangeMotionState
```

Dependencies: ROS2 Humble, rclcpp, interface_protocol, yaml-cpp, LCM, and the
existing SDK's generated `TaskState.hpp` header. CMake finds the header in the
repository's `core/include/data` without modifying it. If this folder is moved
out of the repo, supply its location with
`--cmake-args -DSDK_LCM_INCLUDE_DIR=/path/to/sdk/core/include/data`.

Build artifacts stay under this folder's `build/`, `install/`, and `log/`.
The SDK's `./build.sh` does not build this separate package.

The installed executable records the LCM library directory found at build time
in its runtime search path, together with its own package library directory. If
an older build reports `liblcm.so.1: cannot open shared object file`, rebuild this
package using the commands above. For a temporary fix in the current terminal:

```bash
export LD_LIBRARY_PATH="/opt/engineai_robotics_third_party/lib:${LD_LIBRARY_PATH:-}"
```

When moving the executable to another machine, install the same runtime
dependencies there. If its LCM library directory differs, rebuild there or add
the correct directory to `LD_LIBRARY_PATH`.

## Run with Mujoco first

Start the simulator and existing SDK with `pm01_edu`, `active_mode: sim`, domain
70, and localhost-only ROS discovery using the topic guide. Do not use the
hardware launcher for this test. Disable virtual gamepad sending for ordinary
service calls.

In another **container terminal**, from the repository root:

```bash
source /opt/ros/humble/setup.bash
source build/x86_64/ros2_env/install/local_setup.bash
source extensions/motion_state_service/install/local_setup.bash
export ROS_DOMAIN_ID=70
export ROS_LOCALHOST_ONLY=1

ros2 run engineai_motion_service motion_state_service --ros-args \
  -p sdk_motion_config:="$PWD/assets/config/pm01_edu/task_motion/default.yaml" \
  -p lcm_url:='udpm://239.255.76.67:7667?ttl=0'
```

The supplied motion YAML must match the active SDK task configuration. This
server reads it but never edits it. LCM URL must match the simulator/SDK session.
ROS domains do not isolate LCM.

In a separate client container terminal, source the same three environments and
set the same ROS domain/localhost settings. From the repo root:

```bash
source /opt/ros/humble/setup.bash
source build/x86_64/ros2_env/install/local_setup.bash
source extensions/motion_state_service/install/local_setup.bash
export ROS_DOMAIN_ID=70
export ROS_LOCALHOST_ONLY=1
ros2 service list -t
```

Issue calls individually, observing the response and actual FSM state:

```bash
# Global passive request; also use to return to passive between tests.
ros2 service call /motion/change_state \
  engineai_motion_service/srv/ChangeMotionState \
  "{target_motion_name: 'passive'}"

# From passive: enter PD Stand.
ros2 service call /motion/change_state \
  engineai_motion_service/srv/ChangeMotionState \
  "{target_motion_name: 'pd_stand'}"

# From PD Stand: enter walk.
ros2 service call /motion/change_state \
  engineai_motion_service/srv/ChangeMotionState \
  "{target_motion_name: 'walk'}"

# From walk: enter dance.
ros2 service call /motion/change_state \
  engineai_motion_service/srv/ChangeMotionState \
  "{target_motion_name: 'dance'}"

# From passive: idle, which may quickly auto-transition to passive.
ros2 service call /motion/change_state \
  engineai_motion_service/srv/ChangeMotionState \
  "{target_motion_name: 'idle'}"
```

After passive or another completed request, allow at least one second before
starting a new motion request. The existing SDK adapter retains ordinary combos
for up to 500 ms. Confirmation timeout defaults to three seconds.

The service does not add a stop watchdog. Closing a client/server terminal does
not stop the active motion task. Use passive when responsive; physical tests
still require the robot's documented hardware emergency stop.

## Tests

Run isolated integration tests inside the container, from this folder, after
sourcing ROS and the SDK message install:

```bash
colcon test --packages-select engineai_motion_service --event-handlers console_direct+
colcon test-result --verbose
```

The test uses domain 71, localhost discovery, a separate LCM port 17771, and a
mock SDK. It never runs a robot controller. It covers unknown targets, absent
and stale feedback, missing subscribers, forward/backward adjacency, SDK-rule
rejection, no-ops, confirmation, timeout, busy rejection, and passive preemption.

Also verify with the real SDK in Mujoco:

| Test | Expected |
| --- | --- |
| Passive -> walk | Invalid, no command sent |
| Passive -> PD Stand -> walk | Separate calls succeed and confirm |
| PD Stand -> dance | Invalid, no skipped step |
| Dance -> PD Stand | Invalid; go through walk |
| Walk -> PD Stand | Valid reverse step |
| PD Stand -> idle | Rejected by existing SDK rules; use passive -> idle |
| Any state -> passive | Valid, command dispatched if subscriber exists |
| Virtual gamepad blocks ordinary ROS input | Command sent but confirmation times out if target never appears |
| Physical/virtual LB+RB during pending ordinary request | Existing arbiter stop wins; service must not report unobserved target success |
| Passive call while another service request waits | Pending call reports preemption |
| Feedback stops | Ordinary requests reject once stale; passive still attempts dispatch |

Mock tests do not validate hardware stop behavior. Record any unavailable
physical-gamepad test as untested.

## Parameters

| Parameter | Default | Purpose |
| --- | --- | --- |
| `sdk_motion_config` | Empty (required) | Active SDK motion task YAML |
| `service_name` | `/motion/change_state` | Client-facing service |
| `request_topic` | `/motion/state_request` | Existing SDK input topic |
| `lcm_url` | `udpm://239.255.76.67:7667?ttl=0` | Actual-state feedback |
| `feedback_timeout_sec` | `0.5` | Maximum age for validating ordinary requests |
| `transition_timeout_sec` | `3.0` | Wait for observed target; positive and at most 30 seconds |

LCM messages do not carry a producer timestamp or command ID. Freshness measures
local receive time. Confirmation means the requested target was observed after
dispatch, not that the SDK issued a correlated acceptance response. Stop
priority still depends on the existing adapter and arbiter.

## Main PC server and Orin client

Complete the service's Mujoco tests before hardware deployment. The earlier
topic tests alone do not verify this new package.

Run this server beside the modified SDK on **main**, so it can receive that SDK's
LCM state feedback locally. Build for main's architecture; build/install the
matching service interface on **Orin** for its ARM64 client. Source the local
`interface_protocol` and `engineai_motion_service` installs on each computer.
Use domain 69 and `ROS_LOCALHOST_ONLY=0` on both; set the server's LCM URL and YAML
path to the active robot configuration. Do not copy x86_64 binaries to Orin.

Keep the existing factory installation available for rollback. Follow the topic
guide's controlled controller switch, never run two hardware controllers, and
follow the manual's order: **hanger -> PD Stand -> ground -> walk**. This package
does not change or restart `robotics.service`.

## Files in this folder

- `src/engineai_motion_service/srv/ChangeMotionState.srv`: request/response API.
- `src/engineai_motion_service/include/engineai_motion_service/motion_service.hpp`:
  server declaration and synchronized state.
- `src/engineai_motion_service/src/motion_service.cpp`: validation, feedback,
  dispatch, confirmation, and preemption.
- `src/engineai_motion_service/src/main.cpp`: multithreaded server executable.
- `src/engineai_motion_service/test/`: mock SDK integration test and YAML fixture.
- `src/engineai_motion_service/CMakeLists.txt` and `package.xml`: standalone
  colcon/ament build and dependencies.

The interface uses the standard ROS2 custom-service build pattern; see the
[ROS2 rosidl CMake package reference](https://index.ros.org/p/rosidl_cmake/).
