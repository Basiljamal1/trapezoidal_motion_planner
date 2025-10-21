#include <algorithm>
#include <limits>
#include <gtest/gtest.h>

#include "MockSystems.hpp"

namespace trapezoidal_mp {
namespace test_systems {

// System update implementations
void UpdateIntegratorSystem(float& x, float& v, float v_cmd, float dt,
                            const IntegratorParams& params) {
   const float a_eq = (v_cmd - v) / params.tau_v;
   v += a_eq * dt;
   x += v * dt;
}

void UpdateSecondOrderPositionSystem(float& x, float& v, float control_pos,
                                     float dt,
                                     const SecondOrderPositionParams& params) {
   const float a = (params.wn * params.wn) * (control_pos - x) -
                   (2.0f * params.zeta * params.wn) * v;
   v += a * dt;
   x += v * dt;
}

// Generic rollout function implementation
template <SystemType system_type>
std::vector<TrajectorySample> GenericRollout(const TestConfig& config) {
   CommandStatus status{};
   InterpolationConfig cfg{};
   PositionLimits pos_cfg{};
   CommandData data{};

   status.coordinateMaxVelocity = 1.0e6f;
   status.velocity = config.v0;

   data.position = config.x_target;
   data.velocity = config.target_velocity;
   data.accelerationProfile = config.accel_limit;
   data.velocityProfile = config.vel_limit;

   CoordinateFeedback meas{};
   meas.position = config.x0;
   meas.velocity = config.v0;

   std::vector<TrajectorySample> out;
   out.reserve(static_cast<size_t>(config.rollout_options.maxDuration *
                                   config.rollout_options.rateHz) +
               8);

   const float dt = 1.0f / config.rollout_options.rateHz;
   float x = config.x0, v = config.v0, t = 0.0f;
   const int64_t max_steps = static_cast<int64_t>(
          config.rollout_options.maxDuration * config.rollout_options.rateHz);

   for (int64_t i = 0; i < max_steps; ++i) {
      // Feed measured state to controller
      meas.position = x;
      meas.velocity = v;
      status.velocity = v;

      const float v_cmd =
             UpdateCommand(status, cfg, pos_cfg, meas,
                           config.rollout_options.rateHz, data, data.velocity);

      // Update system based on type
      if constexpr (system_type == SystemType::INTEGRATOR) {
         UpdateIntegratorSystem(x, v, v_cmd, dt,
                                config.system_params.integrator);
      } else if constexpr (system_type == SystemType::SECOND_ORDER_POSITION) {
         UpdateSecondOrderPositionSystem(x, v, status.controlPosition, dt,
                                         config.system_params.second_order);
      }

      if (config.rollout_options.recordEveryStep) {
         out.push_back({t, v_cmd, status.controlPosition, x, v,
                        status.trajectoryFinished});
      }

      t += dt;

      // Check for completion
      if constexpr (system_type == SystemType::INTEGRATOR) {
         if (status.trajectoryFinished) {
            if (!config.rollout_options.recordEveryStep) {
               out.push_back({t, v_cmd, status.controlPosition, x, v,
                              status.trajectoryFinished});
            }
            break;
         }
      } else if constexpr (system_type == SystemType::SECOND_ORDER_POSITION) {
         if (status.trajectoryFinished &&
             std::abs(x - config.x_target) <=
                    config.rollout_options.posTolerance &&
             std::abs(v) <= config.rollout_options.velTolerance) {
            if (!config.rollout_options.recordEveryStep) {
               out.push_back({t, v_cmd, status.controlPosition, x, v,
                              status.trajectoryFinished});
            }
            break;
         }
      }
   }

   return out;
}

// Explicit template instantiations
template std::vector<TrajectorySample> GenericRollout<SystemType::INTEGRATOR>(
       const TestConfig& config);
template std::vector<TrajectorySample>
GenericRollout<SystemType::SECOND_ORDER_POSITION>(const TestConfig& config);

// Generic preemption test implementation
template <SystemType system_type>
PreemptLog RunPreemptedTest(const PreemptTestConfig& config) {
   CommandStatus status{};
   InterpolationConfig cfg{};
   PositionLimits pos_cfg{};
   CommandData data{};

   status.coordinateMaxVelocity = 1.0e6f;
   status.velocity = config.v0;

   data.position = config.x_target_1;
   data.velocity = 0.0f;
   data.accelerationProfile = config.accel_limit;
   data.velocityProfile = config.velocity_limit;

   CoordinateFeedback meas{};
   meas.position = config.x0;
   meas.velocity = config.v0;

   const float dt = 1.0f / config.rate_hz;
   const int64_t max_steps =
          static_cast<int64_t>(config.max_duration_s * config.rate_hz);
   std::vector<TrajectorySample> out;
   out.reserve(static_cast<size_t>(max_steps) + 8);

   float x = config.x0, v = config.v0, t = 0.0f;
   const float total_dist =
          std::max(1e-9f, std::abs(config.x_target_1 - config.x0));
   const float trigger_dist = config.progress_fraction * total_dist;

   bool switched = false;
   size_t switch_idx = static_cast<size_t>(-1);

   for (int64_t i = 0; i < max_steps; ++i) {
      // Preempt based on progress toward first target
      const float progress = std::abs(x - config.x0);
      if (!switched && progress >= trigger_dist && !status.trajectoryFinished) {
         data.position = config.x_target_2;
         switched = true;
         switch_idx = static_cast<size_t>(out.size());
      }

      // Feed measured state
      meas.position = x;
      meas.velocity = v;
      status.velocity = v;

      const float v_cmd = UpdateCommand(status, cfg, pos_cfg, meas,
                                        config.rate_hz, data, data.velocity);

      // Update system based on type
      if constexpr (system_type == SystemType::INTEGRATOR) {
         UpdateIntegratorSystem(x, v, v_cmd, dt,
                                config.system_params.integrator);
      } else if constexpr (system_type == SystemType::SECOND_ORDER_POSITION) {
         UpdateSecondOrderPositionSystem(x, v, status.controlPosition, dt,
                                         config.system_params.second_order);
      }

      out.push_back({t, v_cmd, status.controlPosition, x, v,
                     status.trajectoryFinished});
      t += dt;

      const float goal = switched ? config.x_target_2 : config.x_target_1;
      if (status.trajectoryFinished &&
          std::abs(x - goal) <= config.pos_tolerance &&
          std::abs(v) <= config.vel_tolerance) {
         break;
      }
   }

   return {std::move(out), switch_idx, switched};
}

// Explicit template instantiations
template PreemptLog RunPreemptedTest<SystemType::INTEGRATOR>(
       const PreemptTestConfig& config);
template PreemptLog RunPreemptedTest<SystemType::SECOND_ORDER_POSITION>(
       const PreemptTestConfig& config);

// Helper function for preemption assertions
void CheckPreemptionAssertions(const PreemptLog& L, float expected_final_pos,
                               float velocity_limit, float accel_limit,
                               float rate_hz, const char* csv_path) {
   ASSERT_FALSE(L.traj.empty());

   ASSERT_TRUE(L.switched) << "Preemption did not occur before completion.";
   ASSERT_NE(L.switch_index, static_cast<size_t>(-1));
   ASSERT_LT(L.switch_index, L.traj.size());

   const auto last = L.traj.back();

   EXPECT_NEAR(last.actualPosition, expected_final_pos, 5e-3f);
   EXPECT_NEAR(last.actualVelocity, 0.0f, 5e-3f);

   float max_vcmd = 0.0f;
   for (const auto& s : L.traj) {
      max_vcmd = std::max(max_vcmd, std::abs(s.velocityCommands));
   }
   EXPECT_LE(max_vcmd, velocity_limit + 1e-2f);

   if (std::isfinite(accel_limit)) {
      double max_abs_accel = 0.0;
      for (size_t i = 1; i < L.traj.size(); ++i) {
         const double dv = static_cast<double>(L.traj[i].velocityCommands -
                                               L.traj[i - 1].velocityCommands);
         const double a = dv * rate_hz;
         max_abs_accel = std::max(max_abs_accel, std::abs(a));
      }
      EXPECT_LE(max_abs_accel, 2.5 * accel_limit + 1e-2);
   }

   if (csv_path) {
      WriteCsv(csv_path, L.traj);
   }
}

// TestRunner implementations
template <SystemType system_type>
void TestRunner::RunPositionTest(
       const TestConfig& config, const std::string& test_name,
       std::function<void(const std::vector<TrajectorySample>&)> validator) {
   auto traj = GenericRollout<system_type>(config);

   if (validator) {
      validator(traj);
   }

   if (!test_name.empty()) {
      WriteCsv(test_name + ".csv", traj);
      std::cout << "[trapezoidal_mp] Wrote CSV: " << test_name << ".csv\n";
   }
}

template <SystemType system_type>
void TestRunner::RunVelocityTest(
       float rate_hz, float max_duration_s, float accel_limit,
       float velocity_limit,
       const std::vector<std::pair<float, float>>& velocity_steps,
       const SystemParams& system_params, const std::string& csv_filename) {
   CommandStatus status{};
   InterpolationConfig config{};
   PositionLimits pos_cfg{};
   CommandData data{};

   status.coordinateMaxVelocity = 1.0e6f;;
   status.velocity = 0.0f;

   data.position = std::numeric_limits<float>::quiet_NaN();  // velocity mode
   data.accelerationProfile = accel_limit;
   data.velocityProfile = velocity_limit;

   CoordinateFeedback meas{};
   meas.position = 0.0f;
   meas.velocity = 0.0f;

   const float dt = 1.0f / rate_hz;
   const int max_steps = static_cast<int>(max_duration_s * rate_hz);

   float x = 0.0f, v = 0.0f, t = 0.0f;
   std::vector<TrajectorySample> log;
   log.reserve(static_cast<size_t>(max_steps) + 8);

   size_t step_index = 0;
   float current_target =
          velocity_steps.empty() ? 0.0f : velocity_steps[0].second;

   for (int i = 0; i < max_steps; ++i) {
      // Check if we need to switch to next velocity step
      if (step_index + 1 < velocity_steps.size() &&
          t >= velocity_steps[step_index + 1].first) {
         step_index++;
         current_target = velocity_steps[step_index].second;
      }

      // Feed measurements
      meas.position = x;
      meas.velocity = v;
      status.velocity = v;

      const float v_cmd = UpdateCommand(status, config, pos_cfg, meas, rate_hz,
                                        data, current_target);

      // Update system
      if constexpr (system_type == SystemType::INTEGRATOR) {
         UpdateIntegratorSystem(x, v, v_cmd, dt, system_params.integrator);
      } else if constexpr (system_type == SystemType::SECOND_ORDER_POSITION) {
         UpdateSecondOrderPositionSystem(x, v, status.controlPosition, dt,
                                         system_params.second_order);
      }

      log.push_back({t, v_cmd, status.controlPosition, x, v,
                     status.trajectoryFinished});
      t += dt;
   }

   if (!csv_filename.empty()) {
      WriteCsv(csv_filename, log);
   }
}

template <SystemType system_type>
void TestRunner::RunPreemptionTest(const PreemptTestConfig& config,
                                   const std::string& csv_filename) {
   auto log = RunPreemptedTest<system_type>(config);

   CheckPreemptionAssertions(
          log, config.x_target_2, config.velocity_limit, config.accel_limit,
          config.rate_hz,
          csv_filename.empty() ? nullptr : csv_filename.c_str());
}

// Explicit template instantiations for TestRunner
template void TestRunner::RunPositionTest<SystemType::INTEGRATOR>(
       const TestConfig&, const std::string&,
       std::function<void(const std::vector<TrajectorySample>&)>);
template void TestRunner::RunPositionTest<SystemType::SECOND_ORDER_POSITION>(
       const TestConfig&, const std::string&,
       std::function<void(const std::vector<TrajectorySample>&)>);

template void TestRunner::RunVelocityTest<SystemType::INTEGRATOR>(
       float, float, float, float, const std::vector<std::pair<float, float>>&,
       const SystemParams&, const std::string&);
template void TestRunner::RunVelocityTest<SystemType::SECOND_ORDER_POSITION>(
       float, float, float, float, const std::vector<std::pair<float, float>>&,
       const SystemParams&, const std::string&);

template void TestRunner::RunPreemptionTest<SystemType::INTEGRATOR>(
       const PreemptTestConfig&, const std::string&);
template void TestRunner::RunPreemptionTest<SystemType::SECOND_ORDER_POSITION>(
       const PreemptTestConfig&, const std::string&);

}  // namespace test_systems
}  // namespace trapezoidal_mp