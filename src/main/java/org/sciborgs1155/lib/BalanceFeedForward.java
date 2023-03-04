package org.sciborgs1155.lib;

import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;

public class BalanceFeedforward implements Loggable {
  public @Config double kS, kV, kA, kG;

  public BalanceFeedforward(double kS, double kV, double kA, double kG) {
    this.kS = kS;
    this.kV = kV;
    this.kA = kA;
    this.kG = kG;
  }

  public double calculate(double pitch, double velocity, double acceleration) {
    return kS * Math.signum(velocity) + kV * velocity + kA * acceleration + Math.sin(pitch) * kG;
  }

  public double calculate(double pitch, double velocity) {
    return calculate(pitch, velocity, 0);
  }
}
