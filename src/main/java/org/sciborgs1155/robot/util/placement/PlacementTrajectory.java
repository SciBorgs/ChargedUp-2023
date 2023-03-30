package org.sciborgs1155.robot.util.placement;

import java.util.Map;
import java.util.Optional;
import org.sciborgs1155.lib.Trajectory;

public record PlacementTrajectory(
    Trajectory elevator, Trajectory elbow, Trajectory wrist, Parameters params) {

  public Optional<PlacementTrajectory> findTrajectory(
      Map<Integer, PlacementTrajectory> trajectories,
      PlacementState initialPos,
      PlacementState endPos) {

    int hash = new Parameters(initialPos, endPos).hashCode();
    if (trajectories.get(hash) != null) return Optional.of(trajectories.get(hash));
    return Optional.empty();
  }

  public record Parameters(PlacementState start, PlacementState end) {}
}
