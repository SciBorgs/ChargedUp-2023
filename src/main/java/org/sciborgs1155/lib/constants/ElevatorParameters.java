package org.sciborgs1155.lib.constants;

import edu.wpi.first.math.controller.ElevatorFeedforward;

/** Record to store static, voltage, and acceleration gains for elevator DC motor systems */
public record ElevatorParameters(SimpleParameters base, double g) {

  public ElevatorParameters(double s, double g, double v, double a) {
    this(new SimpleParameters(s, v, a), g);
  }

  public ElevatorFeedforward feedforward() {
    return new ElevatorFeedforward(base.s(), g, base.v(), base.a());
  }
}
