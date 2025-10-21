#include "trapezoidal_motion_planner/TrapezoidalMotionPlanner.hpp"

namespace trapezoidal_mp {

// ============================== Utilities ===================================

bool isfinitef(Coord x) {
   return std::isfinite(x);
}

bool isnanf(Coord x) {
   return std::isnan(x);
}

Coord clamp(Coord x, Coord lo, Coord hi) {
   return std::max(lo, std::min(hi, x));
}

Coord sgn(Coord x) {
   return (x > 0) - (x < 0);
}

// Saturate pos to [min,max] if limits are finite. Returns {pos', hit_limit}
std::pair<Coord, bool> saturatePosIfLimitsHit(Coord pos, Coord minLim,
                                              Coord maxLim) {
   bool hit = false;
   if (isfinitef(minLim) && pos < minLim) {
      pos = minLim;
      hit = true;
   }
   if (isfinitef(maxLim) && pos > maxLim) {
      pos = maxLim;
      hit = true;
   }
   return {pos, hit};
}

// ============================== Core Math ====================================

// Decide acceleration sign under accel/vel constraints in position mode.
Coord CalculateAcceleration(
       const CommandData& data,
       Coord a,   // accel mag limit (finite)
       Coord v0,  // current control velocity
       Coord vf,  // desired target velocity
       Coord dx)  // position error (target - control_position)
{
   // Brake first if overspeed wrt velocity limit.
   if (isfinitef(data.velocityProfile) && std::abs(v0) > data.velocityProfile) {
      return std::copysign(a, -v0);
   }

   // Work in a frame where target velocity is zero.
   const Coord vFrame = v0 - vf;

   if ((vFrame * dx) >= 0.0f && dx != 0.0f) {
      // Moving toward target; enough distance to keep accelerating?
      const Coord decel_dist = (vFrame * vFrame) / (2.0f * a);
      if (std::abs(dx) >= decel_dist) {
         if (!isfinitef(data.velocityProfile) ||
             std::abs(v0) < data.velocityProfile) {
            return std::copysign(a, dx);
         } else {
            return 0.0f;
         }
      } else {
         // Start decelerating to hit target velocity
         return std::copysign(a, -vFrame);
      }
   }

   // Moving away → flip direction
   return std::copysign(a, -vFrame);
}

// Velocity-only mode (no position target).
void DoVelocityModeLimits(CommandStatus& status,
                          const InterpolationConfig& /*cfg*/, Coord rateHz,
                          const CommandData& data, Coord velocityDesired) {
   const Coord dt = 1.0f / rateHz;

   Coord v = velocityDesired;
   if (isfinitef(data.velocityProfile)) {
      v = clamp(v, -data.velocityProfile, data.velocityProfile);
   }

   if (isfinitef(data.accelerationProfile)) {
      const Coord dv = v - status.controlVelocity;
      const Coord sign0 = (dv > 0.0f) ? 1.0f : -1.0f;
      const Coord a = data.accelerationProfile * sign0;

      status.controlVelocity += a * dt;

      const Coord sign1 = (v > status.controlVelocity) ? 1.0f : -1.0f;
      if (sign1 != sign0) {
         status.controlVelocity = v;
         status.trajectoryFinished = true;
      }
   } else {
      status.controlVelocity = v;
      status.trajectoryFinished = true;
   }
}

// Position mode with accel AND/OR velocity limits.
void DoPositionModeLimits(CommandStatus& status,
                          const InterpolationConfig& /*cfg*/, Coord rateHz,
                          CommandData& data, Coord velocityDesired) {
   const Coord dt = 1.0f / rateHz;

   Coord vlimmed = velocityDesired;
   if (isfinitef(data.velocityProfile)) {
      vlimmed = clamp(vlimmed, -data.velocityProfile, data.velocityProfile);
   }

   const Coord v0 = status.controlVelocity;
   const Coord vf = vlimmed;
   const Coord dx =
          (isnanf(data.position) ? 0.0f
                                 : (data.position - status.controlPosition));

   // If no accel limit, we can do the "velocity-only" single-step limit while
   // making sure we don't step past the target in one tick.
   if (!isfinitef(data.accelerationProfile)) {
      if (isfinitef(data.velocityProfile)) {
         // Try stepping at limit toward target; if we'd cross, finish this
         // tick.
         const Coord initial_sign = (dx < 0.0f) ? 1.0f : -1.0f;
         status.controlVelocity = -initial_sign * data.velocityProfile;

         const Coord next_dx = dx - status.controlVelocity * dt;
         const Coord final_sign = (next_dx < 0.0f) ? 1.0f : -1.0f;

         if (final_sign != initial_sign) {
            data.position = kNaN;  // consume target
            status.controlVelocity = vf;
            status.trajectoryFinished = true;
         }
      } else {
         // No limits at all
         status.controlVelocity = vf;
         status.trajectoryFinished = true;
      }
      return;
   }

   // Accel-limited update
   const Coord a = data.accelerationProfile;
   const Coord acc = CalculateAcceleration(data, a, v0, vf, dx);
   status.controlVelocity += acc * dt;

   const Coord v1 = status.controlVelocity;

   // If crossed velocity limit during this tick, clamp exactly on it.
   if (isfinitef(data.velocityProfile)) {
      const Coord lo = std::min(std::abs(v0), std::abs(v1));
      const Coord hi = std::max(std::abs(v0), std::abs(v1));
      if (lo < data.velocityProfile && hi > data.velocityProfile) {
         status.controlVelocity = std::copysign(data.velocityProfile, v0);
      }
   }

   const Coord signed_lo = std::min(v0, v1);
   const Coord signed_hi = std::max(v0, v1);

   const bool target_cross = signed_lo <= vf && signed_hi >= vf;
   const bool target_near = std::abs(v1 - vf) < (a * 0.5f * dt);
   const bool position_near = (std::abs(v1) > 0.0f)
                                     ? (std::abs(dx / v1) <= (kNearFac * dt))
                                     : (std::abs(dx) <= (kNearFac * dt));

   if ((target_cross || target_near) && position_near) {
      data.position = kNaN;  // consume target
      status.controlVelocity = vf;
      status.trajectoryFinished = true;
   }
}

void UpdateTrajectory(CommandStatus& status, const InterpolationConfig& cfg,
                      Coord rateHz, CommandData& data, Coord velocityDesired) {
   Coord v = velocityDesired;
   if (isfinitef(data.velocityProfile)) {
      v = clamp(v, -data.velocityProfile, data.velocityProfile);
   }

   if (isnanf(data.position)) {
      DoVelocityModeLimits(status, cfg, rateHz, data, v);
   } else {
      DoPositionModeLimits(status, cfg, rateHz, data, v);
   }
}

// ============================== Public API ===================================

// UpdateCommand: core stepping function.
// Returns the commanded velocity for this tick (units/s).
Coord UpdateCommand(CommandStatus& status, const InterpolationConfig& cfg,
                    const PositionLimits& pos_limits,
                    const CoordinateFeedback& fb, Coord rateHz,
                    CommandData& data,
                    Coord desiredVelocity /*may be NaN → treated as 0*/) {
   if (isnanf(desiredVelocity)) desiredVelocity = 0.0f;

   // Trajectory "done" bookkeeping
   if (!isfinitef(data.velocityProfile) &&
       !isfinitef(data.accelerationProfile)) {
      status.trajectoryFinished = true;
      status.controlVelocity = desiredVelocity;
   } else if (!isnanf(data.position) || !isnanf(desiredVelocity)) {
      status.trajectoryFinished = false;
   }

   // Fast path: position target with no limits → snap to it immediately.
   if (!isnanf(data.position) && !isfinitef(data.velocityProfile) &&
       !isfinitef(data.accelerationProfile)) {
      status.controlPosition = data.position;
      data.position = kNaN;  // consume target
      status.controlVelocity = desiredVelocity;
      status.controlInitialized = true;
   } else if (!status.controlInitialized) {
      // First-time initialization from measured state
      status.controlPosition = fb.position;
      status.controlVelocity =
             (std::abs(status.velocity) <= cfg.recaptureVelocityEps)
                    ? 0.0f
                    : status.velocity;
      status.controlInitialized = true;
   }

   // If not done, advance the controller state machine.
   if (!status.trajectoryFinished) {
      UpdateTrajectory(status, cfg, rateHz, data, desiredVelocity);
   }

   // Safety clamp on controller velocity
   status.controlVelocity =
          clamp(status.controlVelocity, -status.coordinateMaxVelocity,
                status.coordinateMaxVelocity);

   Coord vCommand = status.controlVelocity;

   // Integrate internal control position with commanded velocity
   const Coord dt = 1.0f / rateHz;
   status.controlPosition += vCommand * dt;

   // If a moving target velocity is present *and* we still have a position
   // target, advance that target too (moving goal).
   if (!isnanf(data.position) && !isnanf(desiredVelocity)) {
      data.position += desiredVelocity * dt;
   }

   // Slip clamps on position and velocity (if configured)
   if (isfinitef(cfg.maxPositionSlip) && !data.syntheticTheta) {
      const Coord error = fb.position - status.controlPosition;
      if (error < -cfg.maxPositionSlip) {
         status.controlPosition = fb.position + cfg.maxPositionSlip;
      }
      if (error > cfg.maxPositionSlip) {
         status.controlPosition = fb.position - cfg.maxPositionSlip;
      }
   }
   if (isfinitef(cfg.maxVelocitySlip) && !data.syntheticTheta) {
      const Coord error = status.velocity - status.controlVelocity;
      if (error < -cfg.maxVelocitySlip) {
         status.controlVelocity = status.velocity + cfg.maxVelocitySlip;
      }
      if (error > cfg.maxVelocitySlip) {
         status.controlVelocity = status.velocity - cfg.maxVelocitySlip;
      }
      vCommand = status.controlVelocity;  // keep output consistent if clamped
   }

   // Enforce absolute position bounds (unless told to ignore).
   bool hit_limit = false;
   if (!data.ignorePositionBounds) {
      auto [pos_sat, hit] = saturatePosIfLimitsHit(status.controlPosition,
                                                   pos_limits.positionMin,
                                                   pos_limits.positionMax);
      status.controlPosition = pos_sat;
      hit_limit |= hit;
   }

   // Optional stop-position clamp: if we're moving *away* from the stop, clamp.
   if (isfinitef(data.stop_position)) {
      const Coord away =
             sgn(status.controlPosition - data.stop_position) * vCommand;
      if (away > 0.0f) {
         status.controlPosition = data.stop_position;
         status.controlVelocity = 0.0f;
         status.trajectoryFinished = true;
         data.position = kNaN;  // consume any pending target
         data.velocity = 0.0f;
         hit_limit = true;
      }
   }

   if (hit_limit) {
      vCommand = 0.0f;
      status.controlVelocity = 0.0f;
   }

   return vCommand;
}

// --------------------------- Rollout helpers -----------------------------

// CSV dump: (t, v_cmd, ctrl_pos, x, v, done)
void WriteCsv(const std::string& path,
              const std::vector<TrajectorySample>& traj) {
   std::ofstream os(path);
   os << "t_s,v_cmd,control_pos,actual_pos,actual_vel,done\n";
   for (const auto& s : traj) {
      os << s.t_s << "," << s.velocityCommands << "," << s.controlPosition
         << "," << s.actualPosition << "," << s.actualVelocity << ","
         << (s.trajectoryFinished ? 1 : 0) << "\n";
   }
}

// Legacy wrapper functions - forward to new test_systems implementation
std::vector<TrajectorySample> SecondOrderIntegratorRollout(
       Coord x0, Coord v0, Coord xTarget, Coord accelerationLimit,
       Coord velocityLimit, const RolloutOptions& opt, Coord tauV,
       Coord targetVelocity) {
   // Forward to new modular implementation - this will be included when
   // test_systems.hpp is available For now, keep the original implementation to
   // maintain compatibility
   CommandStatus status{};
   InterpolationConfig cfg{};
   PositionLimits pos_cfg{};
   CommandData data{};

   status.coordinateMaxVelocity = 1.0e6f;  // effectively no clamp for tests
   status.velocity = v0;

   data.position = xTarget;
   data.velocity = targetVelocity;
   data.accelerationProfile = accelerationLimit;
   data.velocityProfile = velocityLimit;

   CoordinateFeedback meas{};
   meas.position = x0;
   meas.velocity = v0;

   std::vector<TrajectorySample> out;
   out.reserve(static_cast<size_t>(opt.maxDuration * opt.rateHz) + 8);
   const Coord dt = 1.0f / opt.rateHz;

   Coord x = x0, v = v0, t = 0.0f;
   const int64_t max_steps = static_cast<int64_t>(opt.maxDuration * opt.rateHz);

   for (int64_t i = 0; i < max_steps; ++i) {
      // Feed measured state to controller
      meas.position = x;
      meas.velocity = v;
      status.velocity = v;

      const Coord v_cmd = UpdateCommand(status, cfg, pos_cfg, meas, opt.rateHz,
                                        data, data.velocity);

      // Plant update: first-order lag on velocity, then integrate position
      const Coord a_eq = (v_cmd - v) / tauV;
      v += a_eq * dt;
      x += v * dt;

      if (opt.recordEveryStep) {
         out.push_back({t, v_cmd, status.controlPosition, x, v_cmd,
                        status.trajectoryFinished});
      }

      t += dt;

      if (status.trajectoryFinished) {
         if (!opt.recordEveryStep) {
            out.push_back({t, v_cmd, status.controlPosition, x, v,
                           status.trajectoryFinished});
         }
         break;
      }
   }

   return out;
}

std::vector<TrajectorySample> SecondOrderPositionPlantRollout(
       Coord x0, Coord v0, Coord xTarget, Coord accelerationLimit,
       Coord velocityLimit, const RolloutOptions& opt, Coord wn, Coord zeta) {
   // Forward to new modular implementation - this will be included when
   // test_systems.hpp is available For now, keep the original implementation to
   // maintain compatibility
   CommandStatus status{};
   InterpolationConfig cfg{};
   PositionLimits pos_cfg{};
   CommandData data{};

   status.coordinateMaxVelocity = 1.0e6f;
   status.velocity = v0;

   data.position = xTarget;
   data.velocity = 0.0f;
   data.accelerationProfile = accelerationLimit;
   data.velocityProfile = velocityLimit;

   CoordinateFeedback meas{};
   meas.position = x0;
   meas.velocity = v0;

   std::vector<TrajectorySample> out;
   out.reserve(static_cast<size_t>(opt.maxDuration * opt.rateHz) + 8);
   const Coord dt = 1.0f / opt.rateHz;

   Coord x = x0, v = v0, t = 0.0f;
   const int64_t max_steps = static_cast<int64_t>(opt.maxDuration * opt.rateHz);

   for (int64_t i = 0; i < max_steps; ++i) {
      meas.position = x;
      meas.velocity = v;
      status.velocity = v;

      const Coord vCommand = UpdateCommand(status, cfg, pos_cfg, meas,
                                           opt.rateHz, data, data.velocity);

      const Coord u = status.controlPosition;
      const Coord a = (wn * wn) * (u - x) - (2.0f * zeta * wn) * v;
      v += a * dt;
      x += v * dt;

      if (opt.recordEveryStep) {
         out.push_back({t, vCommand, status.controlPosition, x, v,
                        status.trajectoryFinished});
      }

      t += dt;

      if (status.trajectoryFinished &&
          std::abs(x - xTarget) <= opt.posTolerance &&
          std::abs(v) <= opt.velTolerance) {
         if (!opt.recordEveryStep) {
            out.push_back({t, vCommand, status.controlPosition, x, v,
                           status.trajectoryFinished});
         }
         break;
      }
   }

   return out;
}

}  // namespace trapezoidal_mp