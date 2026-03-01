# Sentinel

Sentinel is my deterministic mission simulator for small teams of UAVs and UGVs. Each vehicle gets its own process and has to coordinate search, inspection, relay, and delivery tasks while links fail, routes close, and vehicles drop out.

I keep mission truth in a fixed-step simulator and give each agent only its local observations and delivered peer messages. The agents use decentralized CBBA allocation, BehaviorTree.CPP for execution, capability-aware A* routing, and time-window reservations at shared chokepoints. Runs produce event logs that can be replayed byte for byte. [Architecture](docs/architecture.md) has the longer version.

## Build

You'll need CMake, Ninja, a C++20 compiler, Protobuf (including its Python module), Eigen, GoogleTest, and Python 3. The first CMake configure also downloads the pinned BehaviorTree.CPP revision.

Windows with vcpkg:

```powershell
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE="$env:VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake" -DVCPKG_TARGET_TRIPLET=x64-windows-static
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

Linux with system packages:

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

Or build and test the Docker image:

```bash
docker build -t sentinel .
docker run --rm sentinel
```

## Try a mission

This launches the simulator plus one process per vehicle, runs the scenario three times, compares the tick hashes and event logs, and replays each log:

```bash
PYTHONPATH=build/python python3 tools/run_scenario.py \
  --sim build/bin/sentinel_sim \
  --agent build/bin/sentinel_agent \
  --scenario scenarios/compound_failure_cbba.textproto \
  --output out/demo \
  --repeat 3 \
  --require-complete-reassignments
```

There are smaller scenarios for charging, rerouting, reservations, allocation under a clean network, and individual communication faults. The paired fixtures run CBBA and nearest-capable allocation on the same mission.

## Repository map

| Path | What's there |
| --- | --- |
| `sim/`, `core/` | World state, virtual time, failures, network emulation, reservations, logs, and replay |
| `agent/` | Behavior-tree execution and decentralized allocation |
| `planning/` | A*, energy checks, bundle scoring, and chokepoint scheduling |
| `proto/` | Scenarios, process messages, and trace records |
| `runner/` | Native process supervisor used by benchmark workers |
| `tools/` | Scenario generation, orchestration, benchmark runs, and analysis |
| `ros2/sentinel_gazebo/` | Offline playback in Gazebo Harmonic |

## Gazebo playback

Gazebo is a viewer for an exported run. First make a presentation trace from a replayed event log:

```bash
build/bin/sentinel_sim export \
  --scenario scenarios/compound_failure_cbba.textproto \
  --log out/demo/run-0.events.pb \
  --trace out/demo/demo.trace.pb
```

Then, on a ROS 2 Jazzy system with Gazebo Harmonic and `ros_gz` installed:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths ros2 --packages-select sentinel_gazebo
source install/setup.bash
ros2 launch sentinel_gazebo demo.launch.py trace:=$(pwd)/out/demo/demo.trace.pb
```

The adapter reads the recorded observations and moves simple Gazebo models to match them.
