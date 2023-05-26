package org.sciborgs1155.lib.constants;

import edu.wpi.first.math.controller.PIDController;

/** Record to store P, I, and D gains for PID controllers */
public record PIDConstants(double p, double i, double d) {

  public PIDController createPIDController() {
    return new PIDController(p, i, d);
  }

  public com.pathplanner.lib.auto.PIDConstants toPPL() {
    return new com.pathplanner.lib.auto.PIDConstants(p, i, d);
  }
}
