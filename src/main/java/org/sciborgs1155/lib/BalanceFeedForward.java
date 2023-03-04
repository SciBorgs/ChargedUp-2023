package org.sciborgs1155.lib;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class BalanceFeedforward implements Sendable {
  public double kS, kV, kA, kG;

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

  public void setkS(double kS) {
    this.kS = kS;
  }

  public void setkV(double kV) {
    this.kV = kV;
  }

  public void setkA(double kA) {
    this.kA = kA;
  }

  public void setkG(double kG) {
    this.kG = kG;
  }

  public double getkS() {
    return kS;
  }

  public double getkV() {
    return kV;
  }

  public double getkA() {
    return kA;
  }

  public double getkG() {
    return kG;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("kS", this::getkS, this::setkS);
    builder.addDoubleProperty("kV", this::getkV, this::setkV);
    builder.addDoubleProperty("kA", this::getkA, this::setkA);
    builder.addDoubleProperty("kG", this::getkG, this::setkG);
  }
}
