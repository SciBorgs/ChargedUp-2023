package org.sciborgs1155.robot.util.placement;

import java.util.List;
import java.util.Optional;
import org.sciborgs1155.lib.Trajectory;

public record PlacementTrajectory(
    Trajectory elevator, Trajectory elbow, Trajectory wrist, Parameters params) {

  public Optional<PlacementTrajectory> findTrajectory(
      List<PlacementTrajectory> trajectories, PlacementState initialPos, PlacementState endPos) {
    for (var trajectory : trajectories) {
      if (trajectory.params.start.roughlyEquals(initialPos, 0.001)
          && trajectory.params.end.roughlyEquals(endPos, 0.001)) {
        return Optional.of(trajectory);
      }
    }
    return Optional.empty();
  }

  public record Parameters(PlacementState start, PlacementState end) {}
}
