package org.sciborgs1155.lib.constants;

import edu.wpi.first.math.controller.ArmFeedforward;

/** Record to store static, voltage, and acceleration gains for arm DC motor systems */
public record ArmParameters(SimpleParameters base, double g) {

  public ArmParameters(double s, double g, double v, double a) {
    this(new SimpleParameters(s, v, a), g);
  }

  public ArmFeedforward feedforward() {
    return new ArmFeedforward(base.s(), g, base.v(), base.a());
  }
}
