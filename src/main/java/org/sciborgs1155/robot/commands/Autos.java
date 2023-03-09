package org.sciborgs1155.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import java.util.ArrayList;
import java.util.List;
import org.sciborgs1155.robot.subsystems.Drive;
import org.sciborgs1155.robot.subsystems.Intake;
import org.sciborgs1155.robot.util.Vision;

public class Autos implements Loggable {
  @Log private final SendableChooser<Command> autoChooser;

  private final Drive drive;
  private final Placement placement;
  private final Vision vision;
  private final Intake intake;

  public Autos(Drive drive, Placement placement, Vision vision, Intake intake) {
    this.drive = drive;
    this.vision = vision;
    this.intake = intake;
    this.placement = placement;

    autoChooser = new SendableChooser<Command>();
    autoChooser.setDefaultOption("simple drive", simpleDriveAuto());
    autoChooser.addOption("meandering drive", meanderingDriveAuto());
  }

  private final Command simpleDriveAuto() {
    return drive
        .driveToPose(new Pose2d(1, 5, Rotation2d.fromDegrees(0)))
        .andThen(drive.driveToPose(new Pose2d(1, 1, Rotation2d.fromDegrees(0))));
  }

  private final Command meanderingDriveAuto() {
    Pose2d transitionPose = new Pose2d(15, 7, Rotation2d.fromDegrees(0));
    List<Pose2d> poses =
        new ArrayList<Pose2d>(
            List.of(
                new Pose2d(7, 2, Rotation2d.fromDegrees(0)),
                new Pose2d(7, 7, Rotation2d.fromDegrees(75)),
                transitionPose));
    Pose2d endPose = new Pose2d(1, 7, Rotation2d.fromDegrees(20));
    return drive.driveToPoses(poses).andThen(drive.driveToPose(transitionPose, endPose));
  }

  public Command get() {
    return autoChooser.getSelected();
  }
}
