// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.sciborgs1155.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.sciborgs1155.robot.Constants.AutoConstants.Angular;
import org.sciborgs1155.robot.Constants.AutoConstants.Cartesian;
import org.sciborgs1155.robot.Constants.DriveConstants;
import org.sciborgs1155.robot.subsystems.Drivetrain;

public final class Autos implements Sendable {

  private final Drivetrain drive;

  private final SendableChooser<Command> chooser;

  public Autos(Drivetrain drive) {
    this.drive = drive;

    chooser = new SendableChooser<>();
    chooser.setDefaultOption("mobility", mobility());
    chooser.addOption("other", followPath("New Path", false));
  }

  public Command get() {
    return chooser.getSelected();
  }

  private Command mobility() {
    return Commands.run(() -> drive.drive(0.5, 0, 0, false), drive).withTimeout(5);
  }

  private Command followPath(String pathName, boolean useAllianceColor) {
    PIDController x = new PIDController(Cartesian.kP, Cartesian.kI, Cartesian.kD);
    PIDController y = new PIDController(Cartesian.kP, Cartesian.kI, Cartesian.kD);
    PIDController rot = new PIDController(Angular.kP, Angular.kI, Angular.kD);
    PathPlannerTrajectory loadedPath = PathPlanner.loadPath(pathName, new PathConstraints(5, 4));

    drive.resetOdometry(loadedPath.getInitialPose());
    return new PPSwerveControllerCommand(
            loadedPath,
            drive::getPose,
            DriveConstants.KINEMATICS,
            x,
            y,
            rot,
            drive::setModuleStates,
            useAllianceColor,
            drive)
        .andThen(drive.stop());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    chooser.initSendable(builder);
  }
}
