# Trajectory Generator Test Systems

This directory contains the modular test framework for the Trajectory Generator, providing unified testing capabilities for different plant/system types.

## Quick Start

### Using the New Modular System

```cpp
#include "test_systems.hpp"
using namespace otg::test_systems;

// Configure test
TestConfig config;
config.x0 = 0.0f;                    // Initial position
config.v0 = 0.0f;                    // Initial velocity  
config.x_target = 3.0f;              // Target position
config.accel_limit = 1.0f;           // Acceleration limit
config.vel_limit = 0.8f;             // Velocity limit
config.system_params = SystemParams(0.05f); // tau_v for integrator

// Run test
auto result = GenericRollout<SystemType::INTEGRATOR>(config);
```

### System Types

| System Type | Description | Parameters | Use Cases |
|-------------|-------------|------------|-----------|
| `INTEGRATOR` | First-order velocity lag + position integration | `tau_v` (time constant) | Simple actuator models |
| `SECOND_ORDER_POSITION` | Second-order position control | `wn` (natural freq), `zeta` (damping) | Position-controlled systems |

### Example Tests

#### Position Command Test
```cpp
TEST(MyTest, PositionCommand) {
  TestConfig config;
  config.x_target = 5.0f;
  config.vf_target = 0.2f;  // Non-zero final velocity
  config.system_params = SystemParams(0.05f);
  
  auto result = GenericRollout<SystemType::INTEGRATOR>(config);
  
  // Validate results
  EXPECT_TRUE(result.back().trajectory_done);
  EXPECT_NEAR(result.back().actual_position, 5.0f, 1e-3f);
}
```

#### Velocity Mode Test
```cpp
TEST(MyTest, VelocityMode) {
  TestConfig config;
  config.is_velocity_mode = true;
  config.velocity_steps = {{0.0f, 0.5f}, {1.0f, -0.3f}};
  config.system_params = SystemParams(0.05f);
  
  auto result = GenericRollout<SystemType::INTEGRATOR>(config);
  
  // Test passes through velocity targets
}
```

#### Preemption Test
```cpp
TEST(MyTest, Preemption) {
  PreemptTestConfig config;
  config.x_target_1 = 3.0f;    // Initial target
  config.x_target_2 = -1.5f;   // New target after preemption
  config.progress_fraction = 0.3f; // Preempt at 30% progress
  config.system_params = SystemParams(0.05f);
  
  TestRunner::RunPreemptionTest<SystemType::INTEGRATOR>(config);
}
```

## File Structure

```
test/solver_tests/
├── include/
│   ├── test_systems.hpp     # Modular test framework
│   └── Defines.hpp          # Test utilities
├── src/
│   ├── test_systems.cpp     # Implementation
│   └── TrajectoryGeneratorTest.cpp # Refactored tests
└── CMakeLists.txt          # Build configuration
```

## Key Features

- **🔄 Unified Interface**: Single `GenericRollout<SystemType>()` function
- **⚡ Zero Runtime Overhead**: Template specialization with `constexpr if`
- **🎯 Type Safety**: Compile-time system selection
- **🧪 Rich Testing**: Position, velocity, and preemption test patterns
- **📊 Validation**: Built-in constraint checking and performance profiling
- **🔧 Extensible**: Easy to add new system types

## Migration from Legacy Functions

| Legacy Function | New Approach |
|-----------------|--------------|
| `SecondOrderIntegratorRollout()` | `GenericRollout<SystemType::INTEGRATOR>()` |
| `SecondOrderPositionPlantRollout()` | `GenericRollout<SystemType::SECOND_ORDER_POSITION>()` |

Legacy functions are still available but deprecated. They will be removed in a future release.

## Performance

- **Build time**: Minimal impact (explicit template instantiations)
- **Runtime**: Zero overhead abstraction
- **Memory**: Improved locality through unified data structures

## Visualization

Generate plots of test results:
```bash
python3 /code_workspace/visualize_trajectory_tests.py --summary --show
```

## Contributing

When adding new system types:

1. Add enum value to `SystemType`
2. Implement system update function (e.g., `UpdateMySystemType()`)
3. Add `constexpr if` branch in `GenericRollout()`
4. Add explicit template instantiation
5. Update documentation

## Testing

Run all trajectory generator tests:
```bash
cd build/BHMP/test/solver_tests
./solver_tests --gtest_filter=TGTest.*
```