#pragma once

#include <cstdint>
#include <cmath>
#include <limits>
#include <optional>
#include <algorithm>
#include <functional>
#include <vector>
#include <string>
#include <fstream>
#include <cassert>
#include <iostream>

namespace trapezoidal_mp {

// ============================== Constants ===================================

using Coord = float;  // Coordinate scalar type (switch to double if desired)
constexpr Coord kNaN = std::numeric_limits<Coord>::quiet_NaN();
constexpr Coord kHugeVel = 1.0e9f;  // Very large default max velocity clamp
constexpr Coord kNoClamp = kNaN;    // Sentinel for "no limit"
constexpr Coord kNearFac =
       10.0f;  // Time*factor threshold used for "near target" tests

// ============================== Utilities ===================================

bool isfinitef(Coord x);
bool isnanf(Coord x);
Coord clamp(Coord x, Coord lo, Coord hi);
Coord sgn(Coord x);

// Saturate pos to [min,max] if limits are finite. Returns {pos', hit_limit}
std::pair<Coord, bool> saturatePosIfLimitsHit(Coord pos, Coord minLim,
                                              Coord maxLim);

// ============================== Public Types =================================

// Coordinatized feedback from the plant/sensors.
// If velocity is not reliable (e.g., derived from synthetic theta), set
// CommandData::synthetic_theta=true to relax slip checks that use velocity.
struct CoordinateFeedback {
   Coord position = 0.0f;      // measured coordinate (joint or Cartesian)
   Coord velocity = 0.0f;      // measured velocity in coordinate units/s
   bool positionValid = true;  // optional validity flag (not used by core algo)
   enum ErrorCode { kNone = 0 } error = kNone;
};

// Interpolation/observer configuration.
// recapture_velocity_eps (aka "recapture threshold"): when we first initialize
// the controller's state from measured feedback, if |velocity_filt| <= eps,
// we seed control_velocity as 0 (instead of the filtered velocity). This avoids
// tiny residual drift at start. If you *always* want to seed with measured
// velocity, set eps = 0.
struct InterpolationConfig {
   Coord recaptureVelocityEps = 0.05f;  // "recapture threshold" (see above)

   // Optional slip guards between controller and plant.
   // If finite, clamp the *position* error (controller vs measured) to this
   // bound.
   Coord maxPositionSlip = kNoClamp;
   // If finite, clamp the *velocity* error (controller vs measured) to this
   // bound.
   Coord maxVelocitySlip = kNoClamp;
};

struct PositionLimits {
   Coord positionMin = kNoClamp;  // absolute lower bound (coordinate units)
   Coord positionMax = kNoClamp;  // absolute upper bound (coordinate units)
};

// Controller state (mutable across calls).
struct CommandStatus {
   bool controlInitialized = false;  // set after first UpdateCommand()
   Coord controlPosition = kNaN;     // controller's internal setpoint
   Coord controlVelocity = 0.0f;     // controller's internal velocity state

   // Observed velocities (for seeding and slip logic)
   Coord velocity = 0.0f;      // most recent measured velocity

   // Safety clamp on commanded velocity magnitude
   Coord coordinateMaxVelocity = kHugeVel;

   bool trajectoryFinished = true;  // set when target reached (pos+vel)
};

// User command for one coordinate.
// - If `position` is set (finite), we drive to it subject to accel/vel limits.
// - `velocity` is the target/end velocity at the goal (usually 0).
// - If `position` is NaN, we are in "velocity mode" and will track `velocity`
//   subject to accel/vel limits (or snap if no limits).
// Flags:
// - synthetic_theta: if true, slip clamps that depend on measured velocity are
//   relaxed because the velocity may be unreliable (derived).
// - ignore_position_bounds: if true, do not saturate to PositionLimits.
struct CommandData {
   Coord position = kNaN;  // target position (NaN → velocity mode)
   Coord stop_position =
          kNaN;  // optional "hard stop": if moving away from it, clamp here

   Coord velocity = 0.0f;  // desired end/motion velocity (units/s)

   Coord accelerationProfile = kNoClamp;  // max accel magnitude  (units/s^2)
   Coord velocityProfile = kNoClamp;      // max velocity magnitude (units/s)

   bool syntheticTheta = false;        // see above
   bool ignorePositionBounds = false;  // skip PositionLimits saturation
};

// Trajectory logging record (for plotting, debugging)
struct TrajectorySample {
   Coord t_s = 0.0f;
   Coord velocityCommands = 0.0f;  // output of UpdateCommand
   Coord controlPosition = kNaN;   // internal setpoint
   Coord actualPosition = 0.0f;    // plant
   Coord actualVelocity = 0.0f;    // plant
   bool trajectoryFinished = false;
};

// ============================== Core Math ====================================

// Decide acceleration sign under accel/vel constraints in position mode.
Coord CalculateAcceleration(
       const CommandData& data,
       Coord a,    // accel mag limit (finite)
       Coord v0,   // current control velocity
       Coord vf,   // desired target velocity
       Coord dx);  // position error (target - control_position)

// Velocity-only mode (no position target).
void DoVelocityModeLimits(CommandStatus& status, const InterpolationConfig& cfg,
                          Coord rate_hz, const CommandData& data,
                          Coord velocityDesired);

// Position mode with accel AND/OR velocity limits.
void DoPositionModeLimits(CommandStatus& status, const InterpolationConfig& cfg,
                          Coord rateHz, CommandData& data,
                          Coord velocityDesired);

void UpdateTrajectory(CommandStatus& status, const InterpolationConfig& cfg,
                      Coord rateHz, CommandData& data, Coord velocityDesired);

// ============================== Public API ===================================

// UpdateCommand: core stepping function.
// Returns the commanded velocity for this tick (units/s).
Coord UpdateCommand(CommandStatus& status, const InterpolationConfig& cfg,
                    const PositionLimits& posLimits,
                    const CoordinateFeedback& fb, Coord rateHz,
                    CommandData& data,
                    Coord desiredVelocity /*may be NaN → treated as 0*/);

// --------------------------- Rollout helpers -----------------------------
struct RolloutOptions {
   Coord rateHz = 4000.0f;
   Coord maxDuration = 10.0f;
   Coord posTolerance = 1e-1f;
   Coord velTolerance = 1e-1f;
   bool recordEveryStep = true;
};

// CSV dump: (t, v_cmd, ctrl_pos, x, v, done)
void WriteCsv(const std::string& path,
              const std::vector<TrajectorySample>& traj);

// Legacy functions - DEPRECATED: Use test_systems::GenericRollout instead
// 1) Second-order integrator plant:
//    v follows v_cmd with first-order lag (tau_v). x_dot = v.
[[deprecated(
       "Use test_systems::GenericRollout<SystemType::INTEGRATOR> "
       "instead")]] std::vector<TrajectorySample>
SecondOrderIntegratorRollout(Coord x0, Coord v0, Coord xTarget,
                             Coord accelerationLimit,  // may be NaN
                             Coord velocityLimit,      // may be NaN
                             const RolloutOptions& opt = {},
                             Coord tau_v = 0.01f, Coord targetVelocity = 0.0f);

// 2) Second-order position plant:
//    ẍ + 2ζω_n ẋ + ω_n² x = ω_n² u, where u = control_position.
[[deprecated(
       "Use test_systems::GenericRollout<SystemType::SECOND_ORDER_POSITION> "
       "instead")]] std::vector<TrajectorySample>
SecondOrderPositionPlantRollout(Coord x0, Coord v0, Coord xTarget,
                                Coord accelerationLimit,  // may be NaN
                                Coord velocityLimit,      // may be NaN
                                const RolloutOptions& opt = {},
                                Coord wn = 20.0f, Coord zeta = 0.7f);

}  // namespace trapezoidal_mp