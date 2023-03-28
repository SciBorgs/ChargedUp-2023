package org.sciborgs1155.robot.util.placement;

import org.sciborgs1155.lib.Trajectory;

public record PlacementTrajectory(Trajectory elevator, Trajectory elbow, Trajectory wrist) {

  public record Parameters(PlacementState start, PlacementState end) {}
}
