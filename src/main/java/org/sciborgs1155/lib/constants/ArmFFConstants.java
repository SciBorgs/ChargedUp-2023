package org.sciborgs1155.lib.constants;

import org.sciborgs1155.lib.BetterArmFeedforward;

public record ArmFFConstants(double s, double g, double v, double a) {

  public ArmFFConstants(double s, double g, double v) {
    this(s, g, v, 0);
  }

  public BetterArmFeedforward createFeedforward() {
    return new BetterArmFeedforward(s, g, v, a);
  }
}
