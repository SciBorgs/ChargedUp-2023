// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.sciborgs1155.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/** Add your docs here. */
public interface JointIO extends Sendable, AutoCloseable {

  public Rotation2d getRotation();

  public void updateDesiredState(State state);

  public State getDesiredState();

  public void follow(JointIO joint);

  default void initSendable(SendableBuilder builder) {
    // TODO Auto-generated method stub

  }
}
