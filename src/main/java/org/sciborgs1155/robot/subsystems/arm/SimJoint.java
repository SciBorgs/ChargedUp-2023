// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.sciborgs1155.robot.subsystems.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import org.sciborgs1155.lib.constants.PIDConstants;

/** Add your docs here. */
public class SimJoint implements JointIO {

  private final Rotation2d position = new Rotation2d();
  private final State setpoint = new State();

  private PIDConstants elbowConstants;
  private PIDConstants wristConstants;

  private PIDController elbowPID;
  private PIDController wristPID;

  public SimJoint(boolean newWrist) {
    if (newWrist) {
      elbowConstants = this.newElbow;
      wristConstants = this.newWrist;
    } else {
      elbowConstants = this.oldElbow;
      wristConstants = this.oldWrist;
    }
  }

  public Rotation2d getPosition() {
    return position;
  }

  public void setSetpoints(Rotation2d elbowAngle, Rotation2d wristAngle) {}

  public State getSetpoint(Rotation2d goal) {
    return setpoint;
  }

  public void setStopped(boolean stopped) {}

  public void setPID(PIDConstants constants) {}
}
