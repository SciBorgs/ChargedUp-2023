// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.sciborgs1155.robot.commands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import org.sciborgs1155.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import org.sciborgs1155.robot.Constants.DriveConstants;

public final class Autos {
  /** Example static factory for an autonomous command. */
  // public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
  //   return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  // }

  public static CommandBase mobility(Drivetrain drive) {
    return Commands.run(() -> drive.drive(0.5, 0, 0, false), drive).withTimeout(5);
  }

  public static CommandBase followPath(Drivetrain drive, Trajectory path) {
    return new SwerveControllerCommand(path, drive::getPose, DriveConstants.kinematics, null, null, null);
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
