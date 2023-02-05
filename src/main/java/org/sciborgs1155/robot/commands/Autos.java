// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.sciborgs1155.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import java.util.List;
import org.sciborgs1155.robot.Constants.AutoConstants.Angular;
import org.sciborgs1155.robot.Constants.AutoConstants.Cartesian;
import org.sciborgs1155.robot.subsystems.Drive;

public final class Autos implements Sendable {

  private final Drive drive;

  private final SendableChooser<Command> chooser;

  public Autos(Drive drive) {
    this.drive = drive;

    chooser = new SendableChooser<>();
    chooser.setDefaultOption("mobility", mobility());
    chooser.addOption("other", followPath("New Path"));
  }

  private final TrajectoryConfig autoConfig =
      new TrajectoryConfig(Cartesian.MAX_SPEED, Cartesian.MAX_ACCEL)
          .setKinematics(Drive.KINEMATICS);

  public Command get() {
    return chooser.getSelected();
  }

  private Command mobility() {
    return Commands.run(() -> drive.drive(0.5, 0, 0, false), drive).withTimeout(5);
  }

  private Command followPath(List<Pose2d> path) {
    Trajectory generated = TrajectoryGenerator.generateTrajectory(path, autoConfig);

    return followTrajectory(generated);
  }

  private Command followPath(String pathName) {
    PIDController x = new PIDController(Cartesian.kP, Cartesian.kI, Cartesian.kD);
    PIDController y = new PIDController(Cartesian.kP, Cartesian.kI, Cartesian.kD);
    PIDController rot = new PIDController(Angular.kP, Angular.kI, Angular.kD);
    PathPlannerTrajectory loadedPath = PathPlanner.loadPath(pathName, new PathConstraints(5, 4));

    drive.resetOdometry(loadedPath.getInitialPose());
    return new PPSwerveControllerCommand(
            loadedPath,
            drive::getPose,
            Drive.KINEMATICS,
            x,
            y,
            rot,
            drive::setModuleStates,
            false,
            drive)
        .andThen(drive.stop());
  }

  private Command followTrajectory(Trajectory path) {
    PIDController x = new PIDController(Cartesian.kP, Cartesian.kI, Cartesian.kD);
    PIDController y = new PIDController(Cartesian.kP, Cartesian.kI, Cartesian.kD);
    ProfiledPIDController theta =
        new ProfiledPIDController(Angular.kP, Angular.kI, Angular.kD, Angular.CONSTRAINTS);

    drive.resetOdometry(path.getInitialPose());

    return new SwerveControllerCommand(
            path, drive::getPose, Drive.KINEMATICS, x, y, theta, drive::setModuleStates, drive)
        .andThen(drive.stop());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    chooser.initSendable(builder);
  }
}
