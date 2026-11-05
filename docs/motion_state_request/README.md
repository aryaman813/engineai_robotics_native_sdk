# Run and test ROS2 motion state requests

This guide uses `/motion/state_request` to request PM01 (`pm01_edu`) motion
states from a terminal. Requests become gamepad button combinations inside the
input arbiter, then pass through the existing finite-state machine (FSM).
The feature does not bypass transition rules or modify `core/`.

Start with simulation. Hardware instructions are in a separate section below.
Commands assume Bash, ROS2 Humble, and the repository's default x86_64 build.
Replace the example repository path if your container mounts it elsewhere.

## 1. Understand the interface

| Item | Value |
| --- | --- |
| Topic | `/motion/state_request` |
| Message | `interface_protocol/msg/MotionStateRequest` |
| Field | `string target_motion_name` |
| Subscriber node | `motion_state_request_node` |
| QoS | Reliable, volatile, depth 1 |

| Requested state | Synthesized buttons | Notes |
| --- | --- | --- |
| `passive` | LB+RB | Global soft stop through the existing FSM |
| `pd_stand` | LB+A | Request from passive before requesting walk |
| `walk` | LB+B | Passive cannot transition directly to walk |
| `dance` | RB+B | The current PM01 config uses RB+B, not a D-pad combo |
| `idle` | LB+START | Request from passive; automatically transitions to passive |

Dance has an automatic transition to walk. Task entry/exit conditions can affect
when transitions complete. `rl_lab` exists in the motion configuration but is
not accepted by this subscriber.

The node accepts exact, lowercase names. Unknown names are ignored with a
warning. A request is not an acknowledgement that a transition succeeded.
Observe the actual state in SDK logs or the virtual gamepad task-state display.

### Input priority and limitations

Ordinary inputs are applied in this order:

```text
Selected hardware controller -> ROS state request -> virtual gamepad
```

- Hardware selection prefers RC02 before Logitech and locks the selected source.
  The arbiter does not monitor both physical controllers simultaneously.
- A ROS request temporarily replaces the whole input, including stick values.
  State requests do not command walking velocity.
- An actively transmitting virtual gamepad overrides ordinary ROS requests,
  even when its buttons and sticks are neutral. Disable its sending toggle for
  normal terminal tests. Its input expires after approximately 200 ms without
  messages.
- LB+RB observed from selected hardware, virtual input, or a ROS passive request
  wins over ordinary commands. The arbiter emits a clean LB+RB combination and
  cancels ROS input. Blocked requests are discarded rather than replayed later.
- Ordinary ROS combos expire 500 ms after receipt. Requests older than 500 ms
  when consumed are discarded. A stop can cancel this hold immediately; do not
  assume a ROS passive request is held for 500 ms.
- The mailbox holds one pending request, not a sequence of transitions. Pending
  passive requests are protected against replacement by ordinary requests.
- Losing the publisher does not automatically stop the current motion task.
  Input expiry releases the synthesized buttons; it is not a motion watchdog.

Do not publish commands to `/hardware/gamepad_keys`: it is hardware feedback.
ROS passive and virtual LB+RB are software stops, not hardware emergency stops.

## 2. Build on the development laptop

Enter the development container from a laptop host terminal:

```bash
engineai_robotics_env
```

Inside the container, build and check the message:

```bash
cd /home/htx1/Arya/projects_ws/engineai_robotics_native_sdk
./build.sh
source /opt/ros/humble/setup.bash
source build/x86_64/ros2_env/install/setup.bash
ros2 interface show interface_protocol/msg/MotionStateRequest
```

Expected interface:

```text
string target_motion_name
```

Stop at any build error. Build the simulator only after the SDK build succeeds:

```bash
./scripts/build_mujoco.sh
```

The scripts refresh the existing `core/lib` architecture symlink. Do not edit
core source, headers, or precompiled libraries.

## 3. Check the simulation configuration

In `assets/config/pm01_edu/mode.yaml`, select:

```yaml
active_mode: sim
```

In `assets/config/pm01_edu/ros2_bridge/default.yaml`, retain the existing hardware
node configuration and ensure these entries are present:

```yaml
activated_node:
  - hardware_interface_node
  - motion_state_request_node

motion_state_request_node:
  enable: true
  subscribe_topics:
    state_request: /motion/state_request
```

Merge these entries into the existing YAML; do not create duplicate top-level
keys or replace the whole file with this excerpt.

## 4. Start simulation using three terminals

Enter `engineai_robotics_env` separately in each laptop terminal and change to
the repository root. Use ROS domain **70** and localhost-only discovery for
laptop simulation. The robot launcher uses domain **69**.

ROS domain settings do not isolate LCM traffic. Keep the virtual gamepad on the
local simulation LCM URL `udpm://239.255.76.67:7667?ttl=0`.

### Terminal 1: simulator

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=70
export ROS_LOCALHOST_ONLY=1
./scripts/run_mujoco.sh pm01_edu
```

Leave the simulator window open and ensure simulation is running, not paused.

### Terminal 2: SDK controller

Use a fresh container shell so environment paths from other SDK installations
are not inherited.

```bash
source /opt/ros/humble/setup.bash
source build/x86_64/ros2_env/install/setup.bash
export ROS_DOMAIN_ID=70
export ROS_LOCALHOST_ONLY=1
./run.sh pm01_edu
```

The simulator and controller are separate processes. Both must remain running.
Look for this controller log:

```text
MotionStateRequestNode subscribed to /motion/state_request
```

### Terminal 3: publisher

```bash
source /opt/ros/humble/setup.bash
source build/x86_64/ros2_env/install/setup.bash
export ROS_DOMAIN_ID=70
export ROS_LOCALHOST_ONLY=1
ros2 topic info /motion/state_request --verbose
```

Confirm there is a subscriber named `motion_state_request_node` using the
expected message type. Disable virtual gamepad sending before the first tests.

## 5. Publish state requests

Run these individually in terminal 3, observing each resulting FSM state before
continuing. They are a command reference, not a script to run all at once.

```bash
# Passive / soft stop
ros2 topic pub --once /motion/state_request \
  interface_protocol/msg/MotionStateRequest \
  "{target_motion_name: 'passive'}"

# PD Stand (from passive)
ros2 topic pub --once /motion/state_request \
  interface_protocol/msg/MotionStateRequest \
  "{target_motion_name: 'pd_stand'}"

# Walk (after PD Stand)
ros2 topic pub --once /motion/state_request \
  interface_protocol/msg/MotionStateRequest \
  "{target_motion_name: 'walk'}"

# Dance (from PD Stand or walk)
ros2 topic pub --once /motion/state_request \
  interface_protocol/msg/MotionStateRequest \
  "{target_motion_name: 'dance'}"

# Idle (from passive)
ros2 topic pub --once /motion/state_request \
  interface_protocol/msg/MotionStateRequest \
  "{target_motion_name: 'idle'}"
```

Use `--once`, not a continuous publisher, for these tests. Wait for the observed
transition to complete and at least one second between requests. The simulated
robot can fall when active standing control is removed by passive.

## 6. Simulation test checklist

Record observed FSM states, not just successful publication or receipt logs.

| Test | Procedure | Expected result |
| --- | --- | --- |
| Legal transitions | Passive -> PD Stand -> walk | Each transition succeeds |
| Illegal transition | From passive, request walk; wait one second | Remains passive |
| Dance | From PD Stand or walk, request dance | Enters dance; may later return to walk automatically |
| Idle | From passive, request idle | Enters idle, then returns to passive automatically |
| Unknown target | Publish `'{target_motion_name: "unknown"}'` | Warning, no synthesized input |
| ROS stop | From PD Stand or walk, request passive | Enters passive |
| Virtual stop priority | Hold virtual LB+RB while publishing walk | Passive wins |
| No replay | After that conflict, release buttons and disable virtual sending; wait one second | Remains passive |
| ROS stop vs. virtual | Enable virtual input and enter PD Stand; publish ROS passive | Passive wins despite virtual input |
| Ordinary virtual priority | Keep virtual sending active with neutral buttons; from passive request ROS PD Stand | Remains passive; request is discarded |
| Physical stop priority | Hold selected physical controller's LB+RB while publishing an ordinary request | Passive wins; releasing buttons does not replay the request |

For virtual tests, open a fourth container terminal in the repo root:

```bash
python3 tools/virtual_gamepad/virtual_gamepad.py
```

Connect to the local simulation LCM URL above and enable sending. Use LB+A for
PD Stand and LB+RB for passive. Disable sending when returning to ROS-only tests.

If no physical controller is available, mark the physical stop test **not
tested**. Virtual stop tests do not validate the physical device/driver path.

Stop the simulation by pressing Ctrl+C in the controller and simulator
terminals. Closing the publisher terminal alone does not stop the controller.

## 7. Deploy after simulation passes

The following layout describes the installation used during development:

| Machine | Role |
| --- | --- |
| Development laptop | Build and Mujoco tests |
| `pm01-main` | Runs the motion SDK; requires the modified node and adapter |
| `pm01-orin` | Runs the ROS2 publisher; requires the matching ARM64 message package |

Verify host aliases and architecture on your own robot. Do not copy x86_64
binaries to the ARM64 Orin. The main PC must have matching ROS2 and hardware
runtime dependencies, including `/opt/engineai_robotics_hardware`.

### Copy to a staging folder

Staging copies files without activating them. From the laptop **host**, at the
repo root, for a confirmed x86_64 main PC:

```bash
ssh pm01-main 'mkdir -p ~/projects/engineai_state_request_staging/ros2_env'
rsync -av build/x86_64/_install/ \
  pm01-main:~/projects/engineai_state_request_staging/_install/
rsync -av build/x86_64/ros2_env/install/ \
  pm01-main:~/projects/engineai_state_request_staging/ros2_env/install/
rsync -av assets/ \
  pm01-main:~/projects/engineai_state_request_staging/assets/
rsync -av scripts/run_robot.sh scripts/set_imu_tty.sh \
  pm01-main:~/projects/engineai_state_request_staging/
```

Preserve the existing `/apps/engineai_robotics` installation for rollback.
`install.sh` is not used here: it has a different destination, uses
`rsync --delete`, and does not copy the separate ROS2 environment.

### Check staged dependencies on main

In a fresh shell on `pm01-main`:

```bash
cd ~/projects/engineai_state_request_staging
source /opt/ros/humble/setup.bash
source ros2_env/install/local_setup.bash
export LD_LIBRARY_PATH="$PWD/_install/lib:$PWD/_install/engineai_robotics_third_party/lib:$PWD/_install/engineai_robotics_third_party/lib/runtime:/opt/engineai_robotics_hardware/lib:${LD_LIBRARY_PATH:-}"

ros2 interface show interface_protocol/msg/MotionStateRequest
ldd _install/bin/src_executor | grep 'not found'
ldd _install/lib/libsrc_ros2_node.so | grep 'not found'
ldd _install/lib/libsrc_runner_input_command_arbiter.so | grep 'not found'
```

The interface must match, and all three missing-library checks should produce
no output. Resolve errors before launching the staged controller.

### Controlled hardware test

Proceed only after simulation passes and the staged robot configuration is
verified for the specific hardware. Use the manual's order:
**hanger -> PD Stand -> ground -> walk**. Keep the documented hardware emergency
stop accessible. Physical LB+RB remains unverified until tested with a real
selected controller. Do not treat the ROS connection as an emergency stop.

The commands below assume the existing factory service is `robotics.service`,
running `/apps/engineai_robotics/run_robot.sh pm_v2`, and the staged build belongs
to user `user`. Confirm with `systemctl cat robotics.service` first. Other robot
installations may use different services and paths.

With the robot supported on the hanger, stop the existing controller:

```bash
sudo systemctl stop robotics.service
systemctl is-active robotics.service
pgrep -af 'src_executor|run_robot.sh'
```

Expect `inactive` and no remaining controller processes. Do not run two
controllers against the hardware. Killing only the factory executor is
insufficient because the service has `Restart=always`.

Select robot mode only in the staged configuration:

```bash
cd ~/projects/engineai_state_request_staging
cp -n assets/config/pm01_edu/mode.yaml \
  assets/config/pm01_edu/mode.yaml.before_robot_test
sed -i 's/^active_mode:.*/active_mode: robot/' \
  assets/config/pm01_edu/mode.yaml
grep '^active_mode:' assets/config/pm01_edu/mode.yaml
```

From a fresh main-PC terminal, launch manually with the same privileges as the
factory service:

```bash
sudo bash -c '
set -e
cd /home/user/projects/engineai_state_request_staging
source /opt/ros/humble/setup.bash
source ros2_env/install/local_setup.bash
export ROS_DOMAIN_ID=69
export ROS_LOCALHOST_ONLY=0
export ENGINEAI_ROBOTICS_RUNTIME_ENV=robot
exec bash ./run_robot.sh pm01_edu
'
```

Leave the terminal visible. Stop on hardware initialization errors or unexpected
behavior; do not continue publishing requests to diagnose motor faults.

### Publish from Orin

On Orin, use its own matching message installation:

```bash
source /opt/ros/humble/setup.bash
source ~/engineai_ros2_workspace/install/setup.bash
export ROS_DOMAIN_ID=69
export ROS_LOCALHOST_ONLY=0
ros2 interface show interface_protocol/msg/MotionStateRequest
ros2 topic info /motion/state_request --verbose
```

The message must contain `string target_motion_name`, and discovery must show
`motion_state_request_node`. Both computers need compatible ROS2 networking and
domain 69. If the Orin message is absent or different, build the matching
`interface_protocol` package on Orin; do not reuse laptop Python binaries.

Use the commands in section 5 to request passive, then PD Stand while hanging.
Confirm actual FSM state on main. Verify stop behavior before progressing.
Ground and walk tests follow only after the hanging test succeeds and the robot
is positioned according to its manual. Do not test dance on the first hardware
run.

### Return to the factory controller

Support the robot, request passive if the staged controller is responsive, then
press Ctrl+C in its terminal. On main:

```bash
pgrep -af 'src_executor|run_robot.sh'
```

Only after the staged controller has exited:

```bash
sudo systemctl restart robotics.service
systemctl status robotics.service --no-pager
pgrep -af 'src_executor|run_robot.sh'
```

Expect the original `/apps/engineai_robotics/run_robot.sh pm_v2` controller.
Restarting the service can activate hardware, so keep the robot supported and
clear. The staging folder can remain; it is inactive unless launched.
If the service fails:

```bash
sudo journalctl -u robotics.service -n 60 --no-pager
```

## 8. Troubleshooting

| Symptom | Check / action |
| --- | --- |
| Simulator cannot load `libglog.so.3` | Run inside `engineai_robotics_env`, matching the build environment. Do not create a symlink to a different library ABI. |
| `ldd` reports SDK libraries missing | Source ROS2 and set the staged library paths as shown above before checking. |
| `MotionStateRequest` is unknown | Source the generated message install in the current terminal. The local build path is `build/x86_64/ros2_env/install`, not `build/ros2_env/install`. |
| No subscriber / publisher waits | Check SDK startup, activated-node YAML, ROS domain, localhost setting, and discovery/network configuration on both ends. |
| Topic receipt logged, no transition | Check the current FSM state, permitted transitions, virtual gamepad sending, and task entry/exit logs. |
| Hardware topic exists but new subscriber is absent | An old/factory SDK may be running; verify the executor and installation path. |
| Requests do nothing while virtual UI is open | Disable virtual sending; neutral virtual messages still take priority. |
| `active_mode: basic` | This guide uses `sim` and `robot`; do not copy an unrelated deployment's mode without checking its configuration. |
| Unexpected ROS environment | Close the terminal and open a fresh shell (or reconnect SSH). Re-sourcing `.bashrc` does not undo previous overlays; check startup files if they source overlays automatically. |

## 9. Files added and changed for this feature

This feature extends the native SDK originally obtained from EngineAI's GitHub.
It adds a terminal input path alongside the SDK's existing physical and virtual
gamepad paths:

```text
MotionStateRequest.msg
  -> MotionStateRequestNode (ROS2 callback)
  -> MotionStateRequestMailbox (shared, thread-safe handoff)
  -> MotionStateRequestInputAdapter (state name to GamepadInfo)
  -> InputCommandArbiterRunner (priority and stop handling)
  -> existing FSM (permitted transitions and task entry/exit conditions)
```

The inventory below describes our local feature work. It is not a verified
file-by-file comparison with an upstream EngineAI release. In particular, the
message, mailbox, and incomplete subscriber were already in this working copy
when the review began; they were reused or completed rather than recreated.

### Feature files introduced or completed locally

All paths in the tables are relative to the repository root.

| File | Role and work performed |
| --- | --- |
| `src/protocol/interface_protocol/msg/MotionStateRequest.msg` | Defines `string target_motion_name`. Already present at review; reused unchanged as the request message. |
| `src/data/motion_state_request/motion_state_request_mailbox.h` | Passes one pending request between the ROS2 executor and input runner using a mutex. Already present; updated to protect pending passive requests from ordinary requests. |
| `src/ros2_node/include/ros2_node/motion_state_request_node.h` | Declares the subscriber node, subscription, and mailbox handle. Renamed from the misspelled `moition_state_request_node.h`; corrected constructor syntax and `state_request_subscriber_` spelling. |
| `src/ros2_node/src/motion_state_request_node.cc` | Existing subscriber implementation used by the completed feature. Reads the configured topic, validates the five supported target names, and submits requests to the mailbox. |
| `src/runner/input_command_arbiter/include/input_command_arbiter/motion_state_request_input_adapter.h` | New adapter declaration, including lifecycle methods, request expiry state, and `Cancel()`. |
| `src/runner/input_command_arbiter/src/motion_state_request_input_adapter.cc` | New adapter implementation. Converts state names to existing button combinations and unified `GamepadInfo`, expires requests, and supports cancellation. |
| `docs/motion_state_request/README.md` | New guide for building, running, testing, staging, hardware checks, and rollback. |

### Existing SDK files modified to connect the feature

| File | Change and reason |
| --- | --- |
| `src/runner/input_command_arbiter/src/input_command_arbiter_runner.cc` | Registers the ROS adapter before virtual gamepad input. Captures source inputs independently, gives LB+RB precedence, and cancels blocked ROS requests to prevent later replay. |
| `src/runner/input_command_arbiter/CMakeLists.txt` | Adds `${PROJECT_SOURCE_DIR}/src` as a private include directory so the adapter can include `data/motion_state_request/motion_state_request_mailbox.h`. |
| `assets/config/pm01_edu/ros2_bridge/default.yaml` | Activates `motion_state_request_node` and maps its `state_request` subscription to `/motion/state_request`. |

The protocol and adapter CMake files already discover message/source files with
globs, so no explicit per-file source list was needed. The existing ROS2 node
registration macro makes the subscriber discoverable by the bridge manager.

### Existing SDK components reused

- `core/` provides the precompiled FSM and the `GamepadInfo` / `GamepadTool`
  interfaces. No core implementation changes are required.
- `assets/config/pm01_edu/task_motion/default.yaml` remains the source of allowed
  transitions and button bindings; this feature does not change those rules.
- Physical and virtual gamepad adapters remain the existing input sources. The
  runner integrates the new adapter around them.
- `tools/virtual_gamepad/virtual_gamepad.py` and its UI publish LCM gamepad input;
  they are reused to test arbitration and soft-stop behavior.
- `assets/config/pm01_edu/mode.yaml` selects the test/deployment mode (`sim` or
  `robot`). Selecting a mode is a runtime configuration step, not a new message
  or adapter implementation.

Changing only the message file or topic YAML on a robot does not install the
feature. Rebuild and deploy the SDK libraries containing the adapter and node,
plus the generated message package, as described in section 7.

## 10. Implementation reference

- [Message](../../src/protocol/interface_protocol/msg/MotionStateRequest.msg)
- [ROS2 subscriber](../../src/ros2_node/src/motion_state_request_node.cc)
- [Thread-safe mailbox](../../src/data/motion_state_request/motion_state_request_mailbox.h)
- [Input adapter](../../src/runner/input_command_arbiter/src/motion_state_request_input_adapter.cc)
- [Input arbitration and stop guard](../../src/runner/input_command_arbiter/src/input_command_arbiter_runner.cc)
- [ROS2 activation and topic mapping](../../assets/config/pm01_edu/ros2_bridge/default.yaml)
- [FSM bindings and permitted transitions](../../assets/config/pm01_edu/task_motion/default.yaml)
- [Virtual gamepad instructions](../../tools/virtual_gamepad/README.md)

Simulation transitions and virtual priority tests were reported passing during
development. Physical-gamepad stop verification and hardware acceptance must be
recorded separately; they are not established by those simulation results.
