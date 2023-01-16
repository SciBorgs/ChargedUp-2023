// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.sciborgs1155.robot.commands;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import org.sciborgs1155.robot.Constants.AutoConstants;
import org.sciborgs1155.robot.Constants.DriveConstants;
import org.sciborgs1155.robot.subsystems.Drivetrain;

public final class Autos {
  /** Example static factory for an autonomous command. */
  // public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
  // return Commands.sequence(subsystem.exampleMethodCommand(), new
  // ExampleCommand(subsystem));
  // }

  public static CommandBase mobility(Drivetrain drive) {
    return Commands.run(() -> drive.drive(0.5, 0, 0, false), drive).withTimeout(5);
  }

  public static CommandBase followPath(Drivetrain drive, String pathName) {
    return followTrajectory(
        drive, PathPlanner.loadPath(pathName, AutoConstants.MAX_SPEED, AutoConstants.MAX_ACCEL));
  }

  public static CommandBase followTrajectory(Drivetrain drive, Trajectory path) {
    PIDController x = new PIDController(AutoConstants.P_X_CONTROLLER, 0, 0);
    PIDController y = new PIDController(AutoConstants.P_Y_CONTROLLER, 0, 0);
    ProfiledPIDController theta =
        new ProfiledPIDController(
            AutoConstants.P_THETA_CONTROLLER,
            AutoConstants.MAX_SPEED,
            AutoConstants.MAX_ACCEL,
            AutoConstants.THETA_CONTROLLER_CONSTRAINTS);

    return new SwerveControllerCommand(
        path,
        drive::getPose,
        DriveConstants.KINEMATICS,
        x,
        y,
        theta,
        drive::setModuleStates,
        drive);
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
