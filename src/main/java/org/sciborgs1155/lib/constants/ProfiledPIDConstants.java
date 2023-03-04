package org.sciborgs1155.lib.constants;

import edu.wpi.first.math.controller.ProfiledPIDController;

public record ProfiledPIDConstants(double P, double I, double D) {

  public ProfiledPIDController create() {
    return new ProfiledPIDController(P, I, D, null);
  }

  public void set(ProfiledPIDController controller) {
    controller.setP(P);
    controller.setI(I);
    controller.setD(D);
    controller.setConstraints(null);
  }
}
