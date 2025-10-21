#pragma once

#include <trapezoidal_motion_planner/TrapezoidalMotionPlanner.hpp>
#include <vector>
#include <functional>

namespace trapezoidal_mp {
namespace test_systems {

// System types for compile-time selection
enum class SystemType { INTEGRATOR, SECOND_ORDER_POSITION };

// Base system interface for consistent interface
struct SystemState {
   float position = 0.0f;
   float velocity = 0.0f;
   float time = 0.0f;
};

// System parameters for different plant types
struct IntegratorParams {
   float tau_v = 0.01f;  // velocity time constant
};

struct SecondOrderPositionParams {
   float wn = 20.0f;   // natural frequency
   float zeta = 0.7f;  // damping ratio
};

// Unified system parameters
struct SystemParams {
   IntegratorParams integrator;
   SecondOrderPositionParams second_order;

   SystemParams() = default;

   SystemParams(float tau_v) {
      integrator.tau_v = tau_v;
   }

   SystemParams(float wn, float zeta) {
      second_order.wn = wn;
      second_order.zeta = zeta;
   }
};

// Test configuration for rollouts
struct TestConfig {
   float x0 = 0.0f;
   float v0 = 0.0f;
   float x_target = 0.0f;
   float accel_limit = std::numeric_limits<float>::quiet_NaN();
   float vel_limit = std::numeric_limits<float>::quiet_NaN();
   float target_velocity = 0.0f;
   RolloutOptions rollout_options{};
   SystemParams system_params{};
};

// Preemption test configuration
struct PreemptTestConfig {
   float x0 = 0.0f;
   float v0 = 0.0f;
   float x_target_1 = 0.0f;
   float x_target_2 = 0.0f;
   float accel_limit = 1.0f;
   float velocity_limit = 0.8f;
   float rate_hz = 100.0f;
   float max_duration_s = 8.0f;
   float progress_fraction = 0.5f;
   float pos_tolerance = 1e-3f;
   float vel_tolerance = 1e-3f;
   SystemParams system_params{};
};

// Preemption test result
struct PreemptLog {
   std::vector<TrajectorySample> traj;
   size_t switch_index = static_cast<size_t>(-1);
   bool switched = false;
};

// Generic rollout function that works with different system types
template <SystemType system_type>
std::vector<TrajectorySample> GenericRollout(const TestConfig& config);

// Generic preemption test function
template <SystemType system_type>
PreemptLog RunPreemptedTest(const PreemptTestConfig& config);

// System update functions (used internally by templates)
void UpdateIntegratorSystem(float& x, float& v, float v_cmd, float dt,
                            const IntegratorParams& params);
void UpdateSecondOrderPositionSystem(float& x, float& v, float control_pos,
                                     float dt,
                                     const SecondOrderPositionParams& params);

// Helper functions for test assertions
void CheckPreemptionAssertions(const PreemptLog& log, float expected_final_pos,
                               float velocity_limit, float accel_limit,
                               float rate_hz, const char* csv_path = nullptr);

// Test runner for common test patterns
class TestRunner {
   public:
   // Run a simple position command test
   template <SystemType system_type>
   static void RunPositionTest(
          const TestConfig& config, const std::string& test_name,
          std::function<void(const std::vector<TrajectorySample>&)> validator =
                 nullptr);

   // Run a velocity mode test
   template <SystemType system_type>
   static void RunVelocityTest(float rate_hz, float max_duration_s,
                               float accel_limit, float velocity_limit,
                               const std::vector<std::pair<float, float>>&
                                      velocity_steps,  // time, velocity pairs
                               const SystemParams& system_params,
                               const std::string& csv_filename = "");

   // Run preemption tests
   template <SystemType system_type>
   static void RunPreemptionTest(const PreemptTestConfig& config,
                                 const std::string& csv_filename = "");
};

}  // namespace test_systems
}  // namespace trapezoidal_mp