package org.sciborgs1155.lib.constants;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public record ProfiledPIDConstants(PIDConstants pid, TrapezoidProfile.Constraints constraints) {

  public ProfiledPIDController create() {
    return new ProfiledPIDController(pid.P(), pid.I(), pid.D(), constraints);
  }
}
