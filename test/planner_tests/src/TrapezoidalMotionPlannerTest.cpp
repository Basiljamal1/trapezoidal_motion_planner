#include <chrono>
#include <numeric>
#include <spdlog/spdlog.h>

#include <gtest/gtest.h>

#include <trapezoidal_motion_planner/TrapezoidalMotionPlanner.hpp>

static inline float clampf(float x, float lo, float hi) {
   return std::max(lo, std::min(hi, x));
}

// Test helper class to reduce code duplication
class TrajectoryTestHelper {
   public:
   struct TestConfig {
      float rateHz = 100.0f;
      float maxDurationS = 12.0f;
      float x0 = 0.0f;
      float v0 = 0.0f;
      float xTarget = 0.0f;
      float aLimit = 1.0f;
      float vLimit = 0.8f;
      float vfTarget = 0.0f;
      float tauV = 0.05f;
      bool isVelocityMode = false;
      std::vector<std::pair<float, float>>
             vSteps;  // time, velocity pairs for velocity mode
   };

   struct TestResult {
      std::vector<trapezoidal_mp::TrajectorySample> trajectory;
      bool done = false;
      float tDone = 0.0f;
      float vcmdAtDone = 0.0f;
      std::vector<float> hitTimes;  // for velocity mode step tests
   };

   static TestResult RunIntegratorTest(const TestConfig& config) {
      using namespace trapezoidal_mp;

      const float dt = 1.0f / config.rateHz;
      const int maxSteps =
             static_cast<int>(config.maxDurationS * config.rateHz);

      // Controller setup
      CommandStatus status{};
      InterpolationConfig cfg{};
      PositionLimits pos_cfg{};
      CommandData data{};

      status.coordinateMaxVelocity = 1.0e6f;
      status.velocity = config.v0;

      if (config.isVelocityMode) {
         data.position = std::numeric_limits<float>::quiet_NaN();
      } else {
         data.position = config.xTarget;
         data.velocity = config.vfTarget;
      }
      data.accelerationProfile = config.aLimit;
      data.velocityProfile = config.vLimit;

      CoordinateFeedback meas{};
      meas.position = config.x0;
      meas.velocity = config.v0;

      // Plant state
      float x = config.x0, v = config.v0, t = 0.0f;

      TestResult result;
      result.trajectory.reserve(static_cast<size_t>(maxSteps) + 8);

      size_t step_index = 0;

      for (int i = 0; i < maxSteps; ++i) {
         // Update velocity target for velocity mode
         if (config.isVelocityMode && !config.vSteps.empty()) {
            // Check if we need to switch to next velocity step
            if (step_index + 1 < config.vSteps.size() &&
                t >= config.vSteps[step_index + 1].first) {
               step_index++;
            }
            data.velocity = config.vSteps[step_index].second;
         }

         // Feed measurements
         meas.position = x;
         meas.velocity = v;
         status.velocity = v;

         const float v_cmd = UpdateCommand(status, cfg, pos_cfg, meas,
                                           config.rateHz, data, data.velocity);

         // Plant update (integrator with velocity lag)
         const float a_eq = (v_cmd - v) / config.tauV;
         v += a_eq * dt;
         x += v * dt;

         result.trajectory.push_back({t, v_cmd, status.controlPosition, x, v,
                                      status.trajectoryFinished});

         // Track first "done" moment
         if (!result.done && status.trajectoryFinished) {
            result.done = true;
            result.tDone = t;
            result.vcmdAtDone = v_cmd;
         }

         // Track velocity step hits for velocity mode
         if (config.isVelocityMode && step_index < config.vSteps.size()) {
            const float target_v = config.vSteps[step_index].second;
            if (result.hitTimes.size() <= step_index &&
                std::abs(v_cmd - target_v) <= 0.01f) {
               result.hitTimes.push_back(t);
            }
         }

         t += dt;
      }

      return result;
   }

   static void ValidateBasicConstraints(const TestResult& result,
                                        const TestConfig& config) {
      ASSERT_FALSE(result.trajectory.empty());

      // Velocity limit check
      float maxAbsVcmd = 0.0f;
      for (const auto& s : result.trajectory) {
         maxAbsVcmd = std::max(maxAbsVcmd, std::abs(s.velocityCommands));
      }
      EXPECT_LE(maxAbsVcmd, config.vLimit + 1e-2f);

      // Acceleration limit check
      double maxAbsCmd = 0.0;
      for (size_t i = 1; i < result.trajectory.size(); ++i) {
         const double dv =
                static_cast<double>(result.trajectory[i].velocityCommands -
                                    result.trajectory[i - 1].velocityCommands);
         const double a = dv * config.rateHz;
         maxAbsCmd = std::max(maxAbsCmd, std::abs(a));
      }
      EXPECT_LE(maxAbsCmd, 2.5 * config.aLimit + 1e-2);
   }
};

TEST(TGTest, GenerateTrajectoriesTest) {
   try {
      trapezoidal_mp::RolloutOptions opt;
      opt.rateHz = 200.0f;
      opt.maxDuration = 6.0f;

      const int numberOfRuns = 1000;
      // Lambda to profile and print timing statistics
      auto profileRollout = [&](const std::string& name, auto&& rolloutFunc) {
         std::vector<double> timings;
         std::vector<trapezoidal_mp::TrajectorySample> traj;
         for (int i = 0; i < numberOfRuns; ++i) {
            auto start = std::chrono::high_resolution_clock::now();
            traj = rolloutFunc();
            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> duration = end - start;
            timings.push_back(duration.count());
            traj.clear();
         }
         const double sum =
                std::accumulate(timings.begin(), timings.end(), 0.0);
         const double mean = sum / timings.size();
         const double min = *std::min_element(timings.begin(), timings.end());
         const double max = *std::max_element(timings.begin(), timings.end());
         const double sq_sum = std::inner_product(
                timings.begin(), timings.end(), timings.begin(), 0.0);
         const double stdev = std::sqrt(sq_sum / timings.size() - mean * mean);

         spdlog::info("{} timing statistics over {} runs:", name, numberOfRuns);
         spdlog::info("Average time: {} ms", mean);
         spdlog::info("Min time: {} ms", min);
         spdlog::info("Max time: {} ms", max);
         spdlog::info("Standard deviation: {} ms", stdev);
      };

      // Profile traj1 - legacy function for backward compatibility
      profileRollout("SecondOrderIntegratorRollout", [&]() {
         return trapezoidal_mp::SecondOrderIntegratorRollout(
                0.0f, 0.0f, 3.0f, 1.0f, 0.8f, opt, 0.01f);
      });

      // Profile traj2 - legacy function for backward compatibility
      profileRollout("SecondOrderPositionPlantRollout", [&]() {
         return trapezoidal_mp::SecondOrderPositionPlantRollout(
                0.0f, 0.0f, 3.0f, 1.0f, 0.8f, opt, 20.0f, 0.7f);
      });

      const auto traj1 = trapezoidal_mp::SecondOrderIntegratorRollout(
             0.0f, 0.0f, 3.0f, 1.0f, 0.8f, opt, 0.01f);
      trapezoidal_mp::WriteCsv("output_second_order_integrator.csv", traj1);

      const auto traj2 = trapezoidal_mp::SecondOrderPositionPlantRollout(
             0.0f, 0.0f, 3.0f, 1.0f, 0.8f, opt, 20.0f, 0.7f);
      trapezoidal_mp::WriteCsv("output_second_order_position.csv", traj2);

      std::cout << "[trapezoidal_mp] Wrote CSV: "
                   "output_second_order_integrator.csv\n";
      std::cout << "[trapezoidal_mp] Wrote CSV: "
                   "output_second_order_position.csv\n";
   } catch (...) {
      std::cerr << "[trapezoidal_mp] Unit tests failed (assert).\n";
   }
}

// ---------------- Preemption test helpers (float-based) ----------------------

struct PreemptLog {
   std::vector<trapezoidal_mp::TrajectorySample> traj;
   size_t switch_index = static_cast<size_t>(-1);
   bool switched = false;
};

static PreemptLog RunPreemptedSecondOrderIntegrator(
       float x0, float v0, float xTarget, float xTarget2,
       float accelerationLimit, float velocityLimit, float rateHz,
       float max_duration_s, float tau_v, float progress_fraction,
       float pos_tol = 1e-3f, float vel_tol = 1e-3f) {
   using namespace trapezoidal_mp;

   CommandStatus status{};
   InterpolationConfig cfg{};
   PositionLimits pos_cfg{};
   CommandData data{};

   status.coordinateMaxVelocity = 1.0e6f;
   status.velocity = v0;

   data.position = xTarget;  // initial goal
   data.velocity = 0.0f;     // final velocity goal
   data.accelerationProfile = accelerationLimit;
   data.velocityProfile = velocityLimit;

   CoordinateFeedback meas{};
   meas.position = x0;
   meas.velocity = v0;

   const float dt = 1.0f / rateHz;
   const int64_t maxSteps = static_cast<int64_t>(max_duration_s * rateHz);
   std::vector<TrajectorySample> out;
   out.reserve(static_cast<size_t>(maxSteps) + 8);

   float x = x0, v = v0, t = 0.0f;
   const float totalDist = std::max(1e-9f, std::abs(xTarget - x0));
   const float triggerDist = progress_fraction * totalDist;

   bool switched = false;
   size_t switch_idx = static_cast<size_t>(-1);

   for (int64_t i = 0; i < maxSteps; ++i) {
      // Preempt based on progress toward first target
      const float progress = std::abs(x - x0);
      if (!switched && progress >= triggerDist && !status.trajectoryFinished) {
         data.position = xTarget2;
         switched = true;
         switch_idx = static_cast<size_t>(out.size());
      }

      // Feed measured state
      meas.position = x;
      meas.velocity = v;
      status.velocity = v;

      const float v_cmd = UpdateCommand(status, cfg, pos_cfg, meas, rateHz,
                                        data, data.velocity);

      // Plant update
      const float a_eq = (v_cmd - v) / tau_v;
      v += a_eq * dt;
      x += v * dt;

      out.push_back({t, v_cmd, status.controlPosition, x, v,
                     status.trajectoryFinished});
      t += dt;

      const float goal = switched ? xTarget2 : xTarget;
      if (status.trajectoryFinished && std::abs(x - goal) <= pos_tol &&
          std::abs(v) <= vel_tol) {
         break;
      }
   }

   return {std::move(out), switch_idx, switched};
}

static PreemptLog RunPreemptedSecondOrderPosition(
       float x0, float v0, float xTarget1, float xTarget2, float accelLimit,
       float velocityLimit, float rateHz, float maxDurationS, float wn,
       float zeta, float progressFraction, float pTol = 1e-3f,
       float vTol = 1e-3f) {
   using namespace trapezoidal_mp;

   CommandStatus status{};
   InterpolationConfig cfg{};
   PositionLimits pos_cfg{};
   CommandData data{};

   status.coordinateMaxVelocity = 1.0e6f;
   status.velocity = v0;

   data.position = xTarget1;
   data.velocity = 0.0f;
   data.accelerationProfile = accelLimit;
   data.velocityProfile = velocityLimit;

   CoordinateFeedback meas{};
   meas.position = x0;
   meas.velocity = v0;

   const float dt = 1.0f / rateHz;
   const int64_t maxSteps = static_cast<int64_t>(maxDurationS * rateHz);
   std::vector<TrajectorySample> out;
   out.reserve(static_cast<size_t>(maxSteps) + 8);

   float x = x0, v = v0, t = 0.0f;
   const float total_dist = std::max(1e-9f, std::abs(xTarget1 - x0));
   const float trigger_dist = progressFraction * total_dist;

   bool switched = false;
   size_t switch_idx = static_cast<size_t>(-1);

   for (int64_t i = 0; i < maxSteps; ++i) {
      const float progress = std::abs(x - x0);
      if (!switched && progress >= trigger_dist && !status.trajectoryFinished) {
         data.position = xTarget2;
         switched = true;
         switch_idx = static_cast<size_t>(out.size());
      }

      meas.position = x;
      meas.velocity = v;
      status.velocity = v;

      const float v_cmd = UpdateCommand(status, cfg, pos_cfg, meas, rateHz,
                                        data, data.velocity);

      const float u = status.controlPosition;
      const float a = (wn * wn) * (u - x) - (2.0f * zeta * wn) * v;
      v += a * dt;
      x += v * dt;

      out.push_back({t, v_cmd, status.controlPosition, x, v,
                     status.trajectoryFinished});
      t += dt;

      const float goal = switched ? xTarget2 : xTarget1;
      if (status.trajectoryFinished && std::abs(x - goal) <= pTol &&
          std::abs(v) <= vTol) {
         break;
      }
   }

   return {std::move(out), switch_idx, switched};
}

static void CheckPreemptionAssertions(const PreemptLog& L,
                                      float expectedFinalPos,
                                      float velocityLimit,
                                      float accelerationLimit, float rateHz,
                                      const char* csvPath = nullptr) {
   using namespace trapezoidal_mp;
   ASSERT_FALSE(L.traj.empty());

   ASSERT_TRUE(L.switched) << "Preemption did not occur before completion.";
   ASSERT_NE(L.switch_index, static_cast<size_t>(-1));
   ASSERT_LT(L.switch_index, L.traj.size());

   const auto last = L.traj.back();

   EXPECT_NEAR(last.actualPosition, expectedFinalPos, 5e-3f);
   EXPECT_NEAR(last.actualVelocity, 0.0f, 5e-3f);

   float max_vcmd = 0.0f;
   for (const auto& s : L.traj)
      max_vcmd = std::max(max_vcmd, std::abs(s.velocityCommands));
   EXPECT_LE(max_vcmd, velocityLimit + 1e-2f);

   if (std::isfinite(accelerationLimit)) {
      double maxAbsAccel = 0.0;
      for (size_t i = 1; i < L.traj.size(); ++i) {
         const double dv = static_cast<double>(L.traj[i].velocityCommands -
                                               L.traj[i - 1].velocityCommands);
         const double a = dv * rateHz;
         maxAbsAccel = std::max(maxAbsAccel, std::abs(a));
      }
      EXPECT_LE(maxAbsAccel, 2.5 * accelerationLimit + 1e-2);
   }

   if (csvPath) {
      trapezoidal_mp::WriteCsv(csvPath, L.traj);
   }
}

TEST(TGTest, PreemptEarly_Integrator) {
   const float rate = 500.0f;
   const float maxS = 8.0f;

   const float x0 = 0.0f, v0 = 0.0f;
   const float xTarget1 = 3.0f;
   const float xTarget2 = -1.5f;  // new command heads back the other way
   const float accLimit = 1.0f, velLimit = 0.8f;

   auto L = RunPreemptedSecondOrderIntegrator(
          x0, v0, xTarget1, xTarget2, accLimit, velLimit, rate, maxS,
          /*tau_v=*/0.05f, /*progress_fraction=*/0.20f);

   CheckPreemptionAssertions(L, xTarget2, velLimit, accLimit, rate,
                             "output_preempt_integrator_early.csv");
}

TEST(TGTest, PreemptMid_Integrator) {
   const float rate = 500.0f;
   const float maxS = 8.0f;

   const float x0 = 0.0f, v0 = 0.0f;
   const float xTarget = 3.0f;
   const float xTarget2 =
          5.0f;  // new command extends further in same direction
   const float accLimit = 1.0f, velLimit = 0.8f;

   auto L = RunPreemptedSecondOrderIntegrator(
          x0, v0, xTarget, xTarget2, accLimit, velLimit, rate, maxS,
          /*tau_v=*/0.05f, /*progress_fraction=*/0.50f);

   CheckPreemptionAssertions(L, xTarget2, velLimit, accLimit, rate,
                             "output_preempt_integrator_mid.csv");
}

TEST(TGTest, PreemptLate_Integrator) {
   const float rate = 100.0f;
   const float maxS = 8.0f;

   const float x0 = 0.0f, v0 = 0.0f;
   const float xTarget = 3.0f;
   const float xTarget2 = 1.0f;  // late change to a nearer point (same side)
   const float accLimit = 1.0f, velLimit = 0.8f;

   const auto L = RunPreemptedSecondOrderIntegrator(
          x0, v0, xTarget, xTarget2, accLimit, velLimit, rate, maxS,
          /*tau_v=*/0.05f, /*progress_fraction=*/0.85f);

   CheckPreemptionAssertions(L, xTarget2, velLimit, accLimit, rate,
                             "output_preempt_integrator_late.csv");
}

TEST(TGTest, PreemptEarly_SecondOrderPositionPlant) {
   const float rate = 100.0f;
   const float maxS = 8.0f;

   const float x0 = 0.0f, v0 = 0.0f;
   const float xTarget = 3.0f;
   const float xTarget2 = -1.5f;
   const float accLimit = 1.0f, velLimit = 0.8f;

   auto L = RunPreemptedSecondOrderPosition(
          x0, v0, xTarget, xTarget2, accLimit, velLimit, rate, maxS,
          /*wn=*/20.0f, /*zeta=*/0.7f, /*progress_fraction=*/0.20f);

   CheckPreemptionAssertions(L, xTarget2, velLimit, accLimit, rate,
                             "output_preempt_secondorder_early.csv");
}

TEST(TGTest, PreemptMid_SecondOrderPositionPlant) {
   const float rate = 100.0f;
   const float maxS = 8.0f;

   const float x0 = 0.0f, v0 = 0.0f;
   const float xTarget = 3.0f;
   const float xTarget2 = 5.0f;
   const float accLimit = 1.0f, velLimit = 0.8f;

   auto L = RunPreemptedSecondOrderPosition(
          x0, v0, xTarget, xTarget2, accLimit, velLimit, rate, maxS,
          /*wn=*/20.0f, /*zeta=*/0.7f, /*progress_fraction=*/0.50f);

   CheckPreemptionAssertions(L, xTarget2, velLimit, accLimit, rate,
                             "output_preempt_secondorder_mid.csv");
}

TEST(TGTest, PreemptLate_SecondOrderPositionPlant) {
   const float rate = 100.0f;
   const float maxS = 8.0f;

   const float x0 = 0.0f, v0 = 0.0f;
   const float xTarget = 3.0f;
   const float xTarget2 = 1.0f;
   const float accLimit = 1.0f, velLimit = 0.8f;

   auto L = RunPreemptedSecondOrderPosition(
          x0, v0, xTarget, xTarget2, accLimit, velLimit, rate, maxS,
          /*wn=*/20.0f, /*zeta=*/0.7f, /*progress_fraction=*/0.85f);

   CheckPreemptionAssertions(L, xTarget2, velLimit, accLimit, rate,
                             "output_preempt_secondorder_late.csv");
}

TEST(TGTest, Integrator_PositionCommand_NonZeroFinalVelocity) {
   // Test configuration
   TrajectoryTestHelper::TestConfig config;
   config.rateHz = 4000.0f;
   config.maxDurationS = 12.0f;
   config.x0 = 0.0f;
   config.v0 = 0.0f;
   config.xTarget = 3.0f;
   config.aLimit = 1.0f;
   config.vLimit = 0.8f;
   config.vfTarget = 0.30f;  // desired final velocity
   config.tauV = 0.05f;

   const float vf_expected =
          clampf(config.vfTarget, -config.vLimit, config.vLimit);

   // Run the test
   auto result = TrajectoryTestHelper::RunIntegratorTest(config);

   // Validate results
   ASSERT_TRUE(result.done)
          << "Trajectory never reported done in position+vf mode.";
   EXPECT_NEAR(result.vcmdAtDone, vf_expected, 1e-2f);

   TrajectoryTestHelper::ValidateBasicConstraints(result, config);

   // By the end, plant velocity should have converged to vf_expected
   ASSERT_FALSE(result.trajectory.empty());
   const auto& last = result.trajectory.back();
   EXPECT_NEAR(last.actualVelocity, vf_expected, 3e-2f);

   // Write CSV for visualization
   trapezoidal_mp::WriteCsv("output_integrator_nonzero_vf.csv",
                            result.trajectory);

   std::cout << "[trapezoidal_mp] Non-zero vf (integrator) done at t ≈ "
             << result.tDone << " s, vf* ≈ " << result.vcmdAtDone << "\n";
}

TEST(TGTest, Integrator_VelocityMode_StepUpThenDown) {
   // Test configuration
   TrajectoryTestHelper::TestConfig config;
   config.rateHz = 100.0f;
   config.maxDurationS = 4.0f;
   config.aLimit = 1.0f;
   config.vLimit = 0.6f;
   config.tauV = 0.05f;
   config.isVelocityMode = true;

   const float vStepUp = 0.6f;
   const float vStepDown = -0.4f;
   const float tSwitch = 1.0f;

   config.vSteps = {{0.0f, vStepUp}, {tSwitch, vStepDown}};

   // Expected times to converge (ideal ramp)
   const float t_to_up = std::abs(vStepUp - 0.0f) / config.aLimit;  // 0.6 s
   const float t_to_down =
          std::abs(vStepDown - vStepUp) / config.aLimit;  // 1.0 s
   const float dt = 1.0f / config.rateHz;

   // Run the test
   auto result = TrajectoryTestHelper::RunIntegratorTest(config);

   // Validate basic constraints
   TrajectoryTestHelper::ValidateBasicConstraints(result, config);

   // Check timing assertions
   ASSERT_GE(result.hitTimes.size(), 1)
          << "Did not reach +v target in velocity mode.";
   EXPECT_NEAR(result.hitTimes[0], t_to_up, 3.0f * dt);

   if (result.hitTimes.size() >= 2) {
      EXPECT_NEAR(result.hitTimes[1] - tSwitch, t_to_down, 3.0f * dt);
   }

   // Final plant velocity should be close to the last target
   ASSERT_FALSE(result.trajectory.empty());
   EXPECT_NEAR(result.trajectory.back().actualVelocity, vStepDown, 3e-2f);

   // Write CSV for visualization
   trapezoidal_mp::WriteCsv("output_integrator_velocity_mode.csv",
                            result.trajectory);

   std::cout << "[trapezoidal_mp] Velocity-mode: hit times: ";
   for (size_t i = 0; i < result.hitTimes.size(); ++i) {
      std::cout << result.hitTimes[i] << "s ";
   }
   std::cout << "\n";
}
