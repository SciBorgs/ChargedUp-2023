package org.sciborgs1155.lib.constants;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class PIDConfigurer implements Sendable {

  private double kP;
  private double kI;
  private double kD;

  public PIDConfigurer(PIDConstants defaults) {
    kP = defaults.p();
    kI = defaults.i();
    kD = defaults.d();
  }

  public void setP(double kP) {
    this.kP = kP;
  }

  public void setI(double kI) {
    this.kI = kI;
  }

  public void setD(double kD) {
    this.kD = kD;
  }

  public double getP() {
    return kP;
  }

  public double getI() {
    return kI;
  }

  public double getD() {
    return kD;
  }

  public PIDConstants get() {
    return new PIDConstants(kP, kI, kD);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("P", this::getP, this::setP);
    builder.addDoubleProperty("I", this::getI, this::setI);
    builder.addDoubleProperty("D", this::getD, this::setD);
  }
}
