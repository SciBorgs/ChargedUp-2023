package org.sciborgs1155.robot.util;

import static org.sciborgs1155.robot.Constants.Field.*;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.ArrayList;
import java.util.List;

public class PPLPathFlipper {
  public static List<PathPlannerTrajectory> pathForAlliance(
      List<PathPlannerTrajectory> path, DriverStation.Alliance alliance) {
    if (alliance == Alliance.Red) {
      return flipPathGroup(path);
    }
    return path;
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
    flippedState.poseMeters =
        new Pose2d(
            FIELD_LENGTH_METERS - state.poseMeters.getX(),
            state.poseMeters.getY(),
            Rotation2d.fromRadians(Math.PI - state.poseMeters.getRotation().getRadians()));
    return flippedState;
  }
}
