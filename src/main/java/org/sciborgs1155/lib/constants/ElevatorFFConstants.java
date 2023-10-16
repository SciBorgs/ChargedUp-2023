package org.sciborgs1155.lib.constants;

import org.sciborgs1155.lib.BetterElevatorFeedforward;

public record ElevatorFFConstants(double s, double g, double v, double a) {

  public ElevatorFFConstants(double s, double g, double v) {
    this(s, g, v, 0);
  }

  public BetterElevatorFeedforward createFeedforward() {
    return new BetterElevatorFeedforward(s, g, v, a);
  }
}
