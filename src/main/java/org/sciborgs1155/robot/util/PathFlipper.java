package org.sciborgs1155.robot.util;

import static org.sciborgs1155.robot.Constants.Field.*;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.ArrayList;
import java.util.List;

public class PathFlipper {
  public static List<PathPlannerTrajectory> pathForAlliance(
      List<PathPlannerTrajectory> path, Alliance alliance) {
    return alliance == Alliance.Red ? flipPathGroup(path) : path;
  }

  public static PathPlannerTrajectory pathForAlliance(
      PathPlannerTrajectory path, Alliance alliance) {
    return alliance == Alliance.Red ? flipPath(path) : path;
  }

  public static Pose2d poseForAlliance(Pose2d pose, Alliance alliance) {
    return alliance == Alliance.Red ? flipPose(pose) : pose;
  }

  public static List<PathPlannerTrajectory> flipPathGroup(List<PathPlannerTrajectory> pathGroup) {
    List<PathPlannerTrajectory> flippedPathGroup = new ArrayList<PathPlannerTrajectory>();
    for (PathPlannerTrajectory path : pathGroup) {
      flippedPathGroup.add(flipPath(path));
    }
    return flippedPathGroup;
  }

  public static PathPlannerTrajectory flipPath(PathPlannerTrajectory path) {
    List<State> flippedStates = new ArrayList<State>();
    for (int i = 0; i < path.getStates().size(); i++) {
      flippedStates.add(flipPPState(path.getState(i)));
    }
    return new PathPlannerTrajectory(
        flippedStates,
        path.getMarkers(),
        path.getStartStopEvent(),
        path.getEndStopEvent(),
        path.fromGUI);
  }

  public static PathPlannerState flipPPState(PathPlannerState state) {
    var flippedState = new PathPlannerState();
    flippedState.accelerationMetersPerSecondSq = state.accelerationMetersPerSecondSq;
    flippedState.angularVelocityRadPerSec = -state.angularVelocityRadPerSec;
    flippedState.curvatureRadPerMeter = -state.curvatureRadPerMeter;
    flippedState.timeSeconds = state.timeSeconds;
    flippedState.velocityMetersPerSecond = state.velocityMetersPerSecond;
    flippedState.holonomicAngularVelocityRadPerSec = -state.holonomicAngularVelocityRadPerSec;
    flippedState.holonomicRotation =
        Rotation2d.fromRadians(Math.PI - state.holonomicRotation.getRadians());
    flippedState.poseMeters = flipPose(state.poseMeters);
        // new Pose2d(
        //     FIELD_LENGTH_METERS - state.poseMeters.getX(),
        //     state.poseMeters.getY(),
        //     Rotation2d.fromRadians(Math.PI - state.poseMeters.getRotation().getRadians()));
    return flippedState;
  }

  public static Pose2d flipPose(Pose2d pose) {
    return new Pose2d(
      FIELD_LENGTH_METERS - pose.getX(),
      pose.getY(),
      Rotation2d.fromRadians(Math.PI - pose.getRotation().getRadians()));
  }
}
