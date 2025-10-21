#pragma once

#include "bh_motion_planning/solvers/SolverDefines.hpp"
#include <chrono>
#include <limits>

static constexpr int INITIAL_STREAMING_UPSAMPLING_FACTOR = 10000;
static constexpr int MIN_M_STREAMING = 4;
static constexpr int INITIAL_TRAJECTORY_UPSAMPLING_FACTOR = 30;
static constexpr int MIN_M_TRAJECTORY = 20;
static constexpr int MIN_M1 = 1;

constexpr auto FILE_PREFIX = "";

inline bh_motion_planning::solvers::SolverInput getFourPointTrajetory() {
   size_t dof = 6;
   bh_motion_planning::solvers::SolverInput solverInput;
   solverInput.dt = std::chrono::nanoseconds(std::chrono::milliseconds(2));
   solverInput.velocityLimits = std::vector<double>(dof, 5.0);
   solverInput.accelerationLimits = std::vector<double>(dof, 20.0);
   solverInput.jerkLimits = std::vector<double>(dof, 1000.0);
   solverInput.startState.positions = std::vector<double>(dof, 0.0);
   solverInput.startState.velocities = std::vector<double>(dof, 0.8);
   solverInput.startState.accelerations = std::vector<double>(dof, 0.3);
   solverInput.sparseTrajectory.jointNames = {"joint1", "joint2", "joint3",
                                              "joint4", "joint5", "joint6"};

   // Create a small trajectory with 4 points
   const size_t trajSize = 4;
   const auto stopVector = std::vector<double>(dof, 0.0);
   const auto noConstraints =
          std::vector<double>(dof, std::numeric_limits<double>::quiet_NaN());

   solverInput.sparseTrajectory.points.resize(trajSize);
   solverInput.sparseTrajectory.points[0].positions =
          std::vector<double>(dof, 0.5);
   solverInput.sparseTrajectory.points[0].velocities =
          std::vector<double>(dof, 1.0);
   solverInput.sparseTrajectory.points[0].accelerations =
          std::vector<double>(dof, 0.5);
   solverInput.sparseTrajectory.points[0].timeFromStart =
          std::chrono::milliseconds(25);

   solverInput.sparseTrajectory.points[1].positions =
          std::vector<double>(dof, 1.5);
   solverInput.sparseTrajectory.points[1].velocities = noConstraints;
   solverInput.sparseTrajectory.points[1].accelerations = noConstraints;
   solverInput.sparseTrajectory.points[1].timeFromStart =
          std::chrono::milliseconds(50);

   solverInput.sparseTrajectory.points[2].positions =
          std::vector<double>(dof, 2.0);
   solverInput.sparseTrajectory.points[2].velocities = noConstraints;
   solverInput.sparseTrajectory.points[2].accelerations = noConstraints;
   solverInput.sparseTrajectory.points[2].timeFromStart =
          std::chrono::milliseconds(75);

   solverInput.sparseTrajectory.points[3].positions =
          std::vector<double>(dof, 2.0);
   solverInput.sparseTrajectory.points[3].velocities = stopVector;
   solverInput.sparseTrajectory.points[3].accelerations = stopVector;
   solverInput.sparseTrajectory.points[3].timeFromStart =
          std::chrono::milliseconds(100);
   return solverInput;
}

inline bh_motion_planning::solvers::SolverInput getDilationTestTrajectory() {
   size_t dof = 6;
   bh_motion_planning::solvers::SolverInput solverInput;
   solverInput.dt = std::chrono::nanoseconds(std::chrono::milliseconds(2));
   solverInput.velocityLimits = std::vector<double>(dof, 5.0);
   solverInput.accelerationLimits = std::vector<double>(dof, 20.0);
   solverInput.jerkLimits = std::vector<double>(dof, 1000.0);
   solverInput.startState.positions = std::vector<double>(dof, 0.0);
   solverInput.startState.velocities = std::vector<double>(dof, 0.8);
   solverInput.startState.accelerations = std::vector<double>(dof, 0.3);
   solverInput.sparseTrajectory.jointNames = {"joint1", "joint_2", "joint_3",
                                              "joint4", "joint5",  "joint6"};

   // Create a small trajectory with 2 points
   const size_t trajSize = 2;
   solverInput.sparseTrajectory.points.resize(trajSize);
   solverInput.sparseTrajectory.points[0].positions =
          std::vector<double>(dof, 0.4);
   solverInput.sparseTrajectory.points[0].velocities =
          std::vector<double>(dof, 1.0);
   solverInput.sparseTrajectory.points[0].accelerations =
          std::vector<double>(dof, 0.5);
   solverInput.sparseTrajectory.points[0].timeFromStart =
          std::chrono::milliseconds(25);

   solverInput.sparseTrajectory.points[1] =
          solverInput.sparseTrajectory.points[0];
   // Update the velocity and acceleration to zero at the end
   solverInput.sparseTrajectory.points[1].velocities =
          std::vector<double>(dof, 0.0);
   solverInput.sparseTrajectory.points[1].accelerations =
          std::vector<double>(dof, 0.0);
   solverInput.sparseTrajectory.points[1].timeFromStart =
          std::chrono::milliseconds(100);
   return solverInput;
}

inline bh_motion_planning::solvers::SolverInput
getPositionStreamingTestTrajectory() {
   bh_motion_planning::solvers::SolverInput solverInput;
   const size_t dof = 6;
   solverInput.dt = std::chrono::nanoseconds(std::chrono::milliseconds(2));
   solverInput.velocityLimits = std::vector<double>(dof, 10.0);
   solverInput.accelerationLimits = std::vector<double>(dof, 1000.0);
   solverInput.jerkLimits = std::vector<double>(dof, 100000.0);
   solverInput.startState.positions = std::vector<double>(dof, 0.0);
   solverInput.startState.velocities = std::vector<double>(dof, 0.0);
   solverInput.startState.accelerations = std::vector<double>(dof, 0.0);
   solverInput.sparseTrajectory.jointNames = {"joint1", "joint2", "joint3",
                                              "joint4", "joint5", "joint6"};

   // Create a small trajectory with 2 points
   solverInput.sparseTrajectory.points.resize(2);
   solverInput.sparseTrajectory.points[0].positions =
          std::vector<double>(dof, 0.025);
   solverInput.sparseTrajectory.points[0].velocities =
          std::vector<double>(dof, 0.0);
   solverInput.sparseTrajectory.points[0].accelerations =
          std::vector<double>(dof, 0.0);
   solverInput.sparseTrajectory.points[0].timeFromStart =
          std::chrono::milliseconds(16);

   solverInput.sparseTrajectory.points[1] =
          solverInput.sparseTrajectory.points[0];
   solverInput.sparseTrajectory.points[1].timeFromStart =
          std::chrono::milliseconds(32);

   return solverInput;
}

inline bh_motion_planning::solvers::SolverInput getSinglePointTestTrajectory() {
   bh_motion_planning::solvers::SolverInput solverInput;
   const size_t dof = 6;
   solverInput.dt = std::chrono::nanoseconds(std::chrono::milliseconds(2));
   solverInput.velocityLimits = std::vector<double>(dof, 0.5);
   solverInput.accelerationLimits = std::vector<double>(dof, 0.5);
   solverInput.jerkLimits = std::vector<double>(dof, 1000.0);
   solverInput.startState.positions = std::vector<double>(dof, 0.0);
   solverInput.startState.velocities = std::vector<double>(dof, 0.0);
   solverInput.startState.accelerations = std::vector<double>(dof, 0.0);
   solverInput.startState.timeFromStart =
          std::chrono::nanoseconds(0);  // Ensure time is zero
   solverInput.sparseTrajectory.jointNames = {"joint1", "joint2", "joint3",
                                              "joint4", "joint5", "joint6"};

   // Create a small trajectory with 1 point.
   solverInput.sparseTrajectory.points.resize(1);
   solverInput.sparseTrajectory.points[0].positions =
          std::vector<double>(dof, 1);
   solverInput.sparseTrajectory.points[0].velocities =
          std::vector<double>(dof, 0.0);
   solverInput.sparseTrajectory.points[0].accelerations =
          std::vector<double>(dof, 0.0);
   solverInput.sparseTrajectory.points[0].timeFromStart =
          std::chrono::seconds(3);  // 100ms

   return solverInput;
}

inline bh_motion_planning::solvers::SolverInput
getSinglePointTestTrajectoryDifferentTargetForEachJoint() {
   bh_motion_planning::solvers::SolverInput solverInput;
   const size_t dof = 6;
   solverInput.dt = std::chrono::nanoseconds(std::chrono::milliseconds(2));
   solverInput.velocityLimits = std::vector<double>(dof, 0.5);
   solverInput.accelerationLimits = std::vector<double>(dof, 0.5);
   solverInput.jerkLimits = std::vector<double>(dof, 1000.0);
   solverInput.startState.positions = std::vector<double>(dof, 0.0);
   solverInput.startState.velocities = std::vector<double>(dof, 0.0);
   solverInput.startState.accelerations = std::vector<double>(dof, 0.0);
   solverInput.startState.timeFromStart =
          std::chrono::nanoseconds(0);  // Ensure time is zero
   solverInput.sparseTrajectory.jointNames = {"joint1", "joint2", "joint3",
                                              "joint4", "joint5", "joint6"};

   // Create a small trajectory with 1 point.
   solverInput.sparseTrajectory.points.resize(1);
   solverInput.sparseTrajectory.points[0].positions =
          std::vector<double>(dof, 1);
   solverInput.sparseTrajectory.points[0].velocities =
          std::vector<double>(dof, 0.0);
   solverInput.sparseTrajectory.points[0].accelerations =
          std::vector<double>(dof, 0.0);
   solverInput.sparseTrajectory.points[0].timeFromStart =
          std::chrono::seconds(3);  // 100ms

   for (size_t i = 0; i < dof; ++i) {
      solverInput.sparseTrajectory.points[0].positions[i] =
             static_cast<double>(i + 1);
   }

   return solverInput;
}

inline bh_motion_planning::solvers::SolverInput
getSinglePointTestTrajectoryOneJointMoves() {
   bh_motion_planning::solvers::SolverInput solverInput;
   const size_t dof = 6;
   solverInput.dt = std::chrono::nanoseconds(std::chrono::milliseconds(2));
   solverInput.velocityLimits = std::vector<double>(dof, 0.5);
   solverInput.accelerationLimits = std::vector<double>(dof, 0.5);
   solverInput.jerkLimits = std::vector<double>(dof, 1000.0);
   solverInput.startState.positions = std::vector<double>(dof, 0.0);
   solverInput.startState.velocities = std::vector<double>(dof, 0.0);
   solverInput.startState.accelerations = std::vector<double>(dof, 0.0);
   solverInput.startState.timeFromStart =
          std::chrono::nanoseconds(0);  // Ensure time is zero
   solverInput.sparseTrajectory.jointNames = {"joint1", "joint2", "joint3",
                                              "joint4", "joint5", "joint6"};

   // Create a small trajectory with 1 point.
   solverInput.sparseTrajectory.points.resize(1);
   solverInput.sparseTrajectory.points[0].positions =
          std::vector<double>(dof, 1);
   solverInput.sparseTrajectory.points[0].velocities =
          std::vector<double>(dof, 0.0);
   solverInput.sparseTrajectory.points[0].accelerations =
          std::vector<double>(dof, 0.0);
   solverInput.sparseTrajectory.points[0].timeFromStart =
          std::chrono::seconds(3);  // 100ms

   for (size_t i = 1; i < dof; ++i) {
      solverInput.sparseTrajectory.points[0].positions[i] =
             solverInput.startState.positions[i];
   }

   return solverInput;
}

inline bh_motion_planning::solvers::SolverInput getSingleVelocityTarget() {
   bh_motion_planning::solvers::SolverInput solverInput;
   const size_t dof = 6;
   const auto velocityLimit = 3.0;
   solverInput.dt = std::chrono::nanoseconds(std::chrono::milliseconds(2));
   solverInput.velocityLimits = std::vector<double>(dof, velocityLimit);
   solverInput.accelerationLimits = std::vector<double>(dof, 5);
   solverInput.jerkLimits = std::vector<double>(dof, 1000.0);
   solverInput.startState.positions = std::vector<double>(dof, 0.0);
   solverInput.startState.velocities = std::vector<double>(dof, 0.0);
   solverInput.startState.accelerations = std::vector<double>(dof, 0.0);
   solverInput.startState.timeFromStart =
          std::chrono::nanoseconds(0);  // Ensure time is zero
   solverInput.sparseTrajectory.jointNames = {"joint1", "joint2", "joint3",
                                              "joint4", "joint5", "joint6"};

   // Create a small trajectory with 1 point.
   solverInput.sparseTrajectory.points.resize(1);
   solverInput.sparseTrajectory.points[0].positions =
          std::vector<double>(dof, std::numeric_limits<double>::quiet_NaN());
   solverInput.sparseTrajectory.points[0].velocities =
          std::vector<double>(dof, velocityLimit / 2);
   solverInput.sparseTrajectory.points[0].accelerations =
          std::vector<double>(dof, 0.0);
   solverInput.sparseTrajectory.points[0].timeFromStart =
          std::chrono::seconds(3);  // 100ms

   return solverInput;
}

inline bh_motion_planning::solvers::SolverInput
getVelocityReferenceStreamTestTrajectory() {
   const size_t dof = 6;
   bh_motion_planning::solvers::SolverInput solverInput;
   solverInput.dt = std::chrono::nanoseconds(std::chrono::milliseconds(2));
   solverInput.velocityLimits = std::vector<double>(dof, 5.0);
   solverInput.accelerationLimits = std::vector<double>(dof, 10.0);
   solverInput.jerkLimits = std::vector<double>(dof, 1000.0);
   solverInput.startState.positions = std::vector<double>(dof, 0.0);
   solverInput.startState.velocities = std::vector<double>(dof, 0.0);
   solverInput.startState.accelerations = std::vector<double>(dof, 0.0);
   solverInput.sparseTrajectory.jointNames = {"joint1"};

   // Create a small trajectory with 2 points
   size_t trajSize = 2;
   solverInput.sparseTrajectory.points.resize(trajSize);
   solverInput.sparseTrajectory.points[0].positions =
          std::vector<double>(dof, std::numeric_limits<double>::quiet_NaN());
   solverInput.sparseTrajectory.points[0].velocities =
          std::vector<double>(dof, 0.1);
   solverInput.sparseTrajectory.points[0].accelerations =
          std::vector<double>(dof, std::numeric_limits<double>::quiet_NaN());
   solverInput.sparseTrajectory.points[0].timeFromStart =
          std::chrono::milliseconds(16);

   solverInput.sparseTrajectory.points[1] =
          solverInput.sparseTrajectory.points[0];
   // Update the velocity and acceleration to zero at the end
   solverInput.sparseTrajectory.points[1].velocities =
          std::vector<double>(dof, 0.0);
   solverInput.sparseTrajectory.points[1].accelerations =
          std::vector<double>(dof, 0.0);
   solverInput.sparseTrajectory.points[1].timeFromStart =
          std::chrono::milliseconds(32);
   return solverInput;
}

inline void outputSolverResults(
       bh_motion_planning::solvers::SolverInput& solverInput,
       bh_motion_planning::solvers::SolverResult& solverResult,
       const std::string& sparseFilename, const std::string& denseFilename) {
   std::unordered_map<std::string, bh_motion_planning::solvers::Trajectory*>
          fileToSolverResult = {
                 {sparseFilename, &solverInput.sparseTrajectory},
                 {denseFilename,
                  &solverResult.solverOutput.value().denseTrajectory}};

   for (const auto& file : fileToSolverResult) {
      bh_motion_planning::solvers::outputCSVTrajectory(*file.second,
                                                       FILE_PREFIX, file.first);
   }
}

inline void outputSolverResults(
       bh_motion_planning::solvers::Trajectory& sparseTrajectory,
       bh_motion_planning::solvers::Trajectory& denseTrajectory,
       const std::string& sparseFilename, const std::string& denseFilename) {
   std::unordered_map<std::string, bh_motion_planning::solvers::Trajectory*>
          fileToSolverResult = {{sparseFilename, &sparseTrajectory},
                                {denseFilename, &denseTrajectory}};

   for (const auto& file : fileToSolverResult) {
      bh_motion_planning::solvers::outputCSVTrajectory(*file.second,
                                                       FILE_PREFIX, file.first);
   }
}