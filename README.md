# Trapezoidal Motion Planner

A high-performance trajectory planner capable of running at extremely fast real-time speeds with preemption capabilities (also known as shortcutting). The planner can operate in both position space and velocity space, making it suitable for a wide range of motion control applications.

## Features

- **⚡ Ultra-High Performance**: Optimized for real-time execution at rates up to 4000+ Hz
- **🎯 Dual Mode Operation**: Position mode for point-to-point moves, velocity mode for continuous motion
- **🔄 Dynamic Preemption**: Change targets mid-trajectory with smooth transitions (shortcutting).
- **⚡ Streaming capability, Interpolation**: The planner is capable of interpolating between commands that arrive at high frequency, to enable smooth, kinematically constrained commands to the joints while acting as an adaptor between two different frequency interfaces.  
- **🛡️ Constraint Enforcement**: Respects acceleration and velocity limits.
- **📊 Built-in Validation**: Comprehensive test suite with performance profiling

## Build Instructions

### Prerequisites
- CMake 3.10 or higher
- C++17 compatible compiler
- GTest (for running tests)

### Building the Project

```bash
# Clone and navigate to project directory
cd trapezoidal_motion_planner

# Create build directory
mkdir -p build
cd build

# Configure with CMake
cmake ..

# Build the project
cmake --build .

# Run tests to generate CSV data
cd test/planner_tests
./planner_tests

# Generate plots (requires matplotlib)
cd ../../../test/planner_tests/py
python3 plot_tg.py
```

## Test Results

The planner has been extensively tested across multiple scenarios to validate performance, constraint compliance, and real-time capabilities. Key validation areas include:

- **Basic Trajectory Generation**: Position and velocity control with different plant models
- **Velocity Mode Operation**: Dynamic velocity tracking with smooth transitions  
- **Preemption Capabilities**: Mid-trajectory target changes with optimal replanning
- **Performance Benchmarks**: Microsecond-level execution times suitable for 4000+ Hz control

### Quick Performance Summary

- **Execution Time**: 0.023ms average (suitable for real-time control)

### Detailed Results

For comprehensive test results including plots, performance analysis, and detailed technical discussion, see:

**📊 [Complete Test Results and Analysis](docs/test-results.md)**

This detailed documentation includes plots of the planner output 

## Usage

### Position Mode

Use position mode for standard point-to-point movements:

```cpp
#include <trapezoidal_motion_planner/TrapezoidalMotionPlanner.hpp>

using namespace trapezoidal_mp;

// Initialize controller state
CommandStatus status{};
InterpolationConfig cfg{};
PositionLimits pos_limits{};

// Configure command
CommandData cmd{};
cmd.position = 5.0f;              // Target position
cmd.velocity = 0.0f;              // Final velocity (typically 0). Otherwise planner maintains this velocity after trajectory is finished. "Feedforward"
cmd.accelerationProfile = 2.0f;     // Max acceleration
cmd.velocityProfile = 1.0f;         // Max velocity

// Feedback from plant/sensors
CoordinateFeedback feedback{};
feedback.position = 0.0f;         // Current position
feedback.velocity = 0.0f;         // Current velocity

const float rate_hz = 1000.0f;    // Control loop frequency

// Main control loop
while (!status.trajectoryFinished) {
    // Update measurements
    feedback.position = /* read from sensors */;
    feedback.velocity = /* read from sensors */;
    
    // Compute velocity command
    float v_cmd = UpdateCommand(
        status, cfg, pos_limits, 
        feedback, rate_hz, cmd, cmd.velocity);
    
    // Send command to actuator
    /* send v_cmd to motor controller */
    /* Optionally integrate to position and send to position motor controller */
    /* pos_cmd += v_cmd * dt;
    
    // Wait for next control cycle
    /* sleep for 1/rate_hz seconds */;
}
```

### Velocity Mode

Use velocity mode for continuous motion tracking:

```cpp
#include <trapezoidal_motion_planner/TrapezoidalMotionPlanner.hpp>

using namespace trapezoidal_mp;

// Initialize controller state
CommandStatus status{};
InterpolationConfig cfg{};
PositionLimits pos_limits{};

// Configure for velocity mode
CommandData cmd{};
cmd.position = kNaN;              // NaN indicates velocity mode
cmd.velocity = 0.5f;              // Target velocity
cmd.accelerationProfile = 2.0f;     // Max acceleration
cmd.velocityProfile = 1.0f;         // Max velocity magnitude

CoordinateFeedback feedback{};
const float rate_hz = 1000.0f;

// Control loop with changing velocity targets
while (/* running */) {
    // Update target velocity as needed
    cmd.velocity = /* new target velocity */;
    
    // Update measurements
    feedback.position = /* read from sensors */;
    feedback.velocity = /* read from sensors */;
    
    // Compute velocity command
    float v_cmd = UpdateCommand(
        status, cfg, pos_limits,
        feedback, rate_hz, cmd, cmd.velocity);
    
    // Send to actuator
    /* send v_cmd to motor controller */;
    
    /* sleep for control cycle */;
}
```

### Dynamic Preemption (Shortcutting)

Change targets during trajectory execution:

```cpp
// Initial setup (position mode)
CommandData cmd{};
cmd.position = 10.0f;             // Initial target
cmd.velocity = 0.0f;
cmd.accelerationProfile = 2.0f;
cmd.velocityProfile = 1.0f;

bool target_changed = false;

while (!status.trajectoryFinished) {
    // Check if we need to change target
    if (!target_changed && /* some condition */) {
        cmd.position = 15.0f;     // New target
        target_changed = true;
        // No need to reinitialize - planner handles preemption automatically
    }
    
    // Continue normal control loop
    float v_cmd = UpdateCommand(
        status, cfg, pos_limits,
        feedback, rate_hz, cmd, cmd.velocity);
    
    /* ... */
}
```

## Key Parameters

### CommandData Structure
- `position`: Target position (set to `kNaN` for velocity mode)
- `velocity`: Target/final velocity
- `accelerationProfile`: Maximum acceleration magnitude
- `velocityProfile`: Maximum velocity magnitude

### Control Loop Requirements
- **Frequency**: 20 Hz minimum, 1000+ Hz recommended for best performance
- **Timing**: Consistent control loop timing is important for optimal results
- **Feedback**: Position required, velocity estimate required. 

## Performance Characteristics

- **Computational Cost**: Extremely low - suitable for hard real-time systems. Compatible for both STM32 devices and high level devices. 
- **Memory Footprint**: Minimal state storage
- **Deterministic**: Guaranteed bounded execution time
- **Thread Safe**: No global state (when using separate status objects). The design is made functional on purpose. 

## Applications

- **Robotic Joint Trajectory Control**: Individual joint trajectory planning. Point to point control. 
- **Teleoperation of Robot Arms and Humanoids**: The planner is very well suited as an adaptor layer between two control loops running at different frequencies. High level planner runs at 200Hz (e.g Cartesian control) while low level joint planner (this) running at 1000Hz, while providing a smooth teleoperation experience given the profile values while obeying joint limit constraints. 

- **Robotic Joint Trajectory Control**: Individual joint trajectory planning for single-axis or per-joint controllers (point-to-point and streaming modes).
- **Teleoperation of Robot Arms and Humanoids**: Ideal as an adapter between control layers running at different rates. For example, a high-level Cartesian planner at ~200 Hz can emit targets that this planner reconciles and streams to a 1000+ Hz joint controller. The planner enforces kinematic constraints and smooths commands to improve teleoperation responsiveness and feel while preserving safety limits.

## Other applicatoins
- **CNC Machine Tools**: Axis motion planning with precise constraints
- **Conveyor Systems**: Speed control with smooth acceleration profiles
- **Camera Gimbals**: Smooth position and velocity tracking
- **General Motion Control**: Any single-axis motion control application
