package org.sciborgs1155.lib;

import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;

public class PlacementTrajectory {

  private List<State> states;

  public static class State {
    public final double elevatorHeight;
    public final Rotation2d elbowAngle;
    public final Rotation2d wristAngle;

    public State(double elevatorHeight, Rotation2d elbowAngle, Rotation2d wristAngle) {
      this.elevatorHeight = elevatorHeight;
      this.elbowAngle = elbowAngle;
      this.wristAngle = wristAngle;
    }
  }
}
