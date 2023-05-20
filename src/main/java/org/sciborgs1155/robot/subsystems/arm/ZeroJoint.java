// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.sciborgs1155.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

/** Add your docs here. */
public class ZeroJoint implements JointIO {
  private State state = new State();

  public Rotation2d getRotation() {}

  public void updateDesiredState(State state) {}

  public State getDesiredState() {}
}
