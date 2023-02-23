package org.sciborgs1155.robot.util;

import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;

public final class PathPlannerHelpers {

  public static PathPoint pose2dToPathPoint(Pose2d pose) {
    return new PathPoint(pose.getTranslation(), pose.getRotation(), pose.getRotation());
  }
}
