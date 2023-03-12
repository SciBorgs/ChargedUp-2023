package org.sciborgs1155.robot.commands;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import java.util.List;
import org.sciborgs1155.lib.PlacementState;
import org.sciborgs1155.lib.Vision;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.commands.Scoring.Alliance;
import org.sciborgs1155.robot.commands.Scoring.GamePiece;
import org.sciborgs1155.robot.commands.Scoring.ScoringHeight;
import org.sciborgs1155.robot.commands.Scoring.Side;
import org.sciborgs1155.robot.subsystems.Drive;
import org.sciborgs1155.robot.subsystems.Intake;

public class Autos implements Loggable {
  @Log private final SendableChooser<Command> autoChooser;

  private final Drive drive;
  private final Placement placement;
  private final Vision vision;
  private final Intake intake;
  private final Scoring scoring;

  public Autos(Drive drive, Placement placement, Vision vision, Intake intake, Scoring scoring) {
    this.drive = drive;
    this.vision = vision;
    this.intake = intake;
    this.placement = placement;
    this.scoring = scoring;

    autoChooser = new SendableChooser<Command>();
    autoChooser.setDefaultOption("simple drive", simpleDrive());
    autoChooser.addOption("meandering drive", meanderingDrive());
    autoChooser.addOption("balance", balance());
    autoChooser.addOption("goofy", goofy());
    autoChooser.addOption("goofyApp", goofyApp());
    autoChooser.addOption("2 cubes low", Commands.run(() -> twoCubesLow().schedule()));
    autoChooser.addOption("score", Commands.run(() -> score()).andThen(drive.stop()));
  }

  private Command simpleDrive() {
    Pose2d end = new Pose2d(1, 5, Rotation2d.fromDegrees(0));
    return drive
        .driveToPose(end)
        .andThen(drive.driveToPose(end, new Pose2d(0, 0, Rotation2d.fromDegrees(0))));
  }

  private Command meanderingDrive() {
    Pose2d transitionPose = new Pose2d(15, 7, Rotation2d.fromDegrees(0));
    List<Pose2d> poses =
        List.of(
            new Pose2d(7, 2, Rotation2d.fromDegrees(0)),
            new Pose2d(7, 7, Rotation2d.fromDegrees(75)),
            transitionPose);
    Pose2d endPose = new Pose2d(1, 7, Rotation2d.fromDegrees(20));
    return drive.driveToPoses(poses).andThen(drive.driveToPose(transitionPose, endPose));
  }

  private Command balance() {
    double tolerance = 5;
    BangBangController balance = new BangBangController(tolerance);
    return Commands.run(() -> drive.drive(balance.calculate(drive.getPitch(), 0), 0, 0, true));
  }

  private Command goofy() {
    drive.resetOdometry(new Pose2d(1, 3, Rotation2d.fromDegrees(0)));
    return drive.driveToPoses(
        List.of(
            new Pose2d(1.87, 3.79, Rotation2d.fromDegrees(180)),
            new Pose2d(2.84, 4.76, Rotation2d.fromDegrees(0)),
            new Pose2d(2.05, 1.99, Rotation2d.fromDegrees(180))));
  }

  private Command goofyApp() {
    return drive.follow("goofy", true, false);
  }

  // i kind of hate having this here but oh well
  private Command autoIntake(PlacementState placementState) {
    return scoring
        .intake(placementState)
        .alongWith(drive.drive(() -> 0.1, () -> 0.1, () -> 0, false))
        .until(intake::isHoldingItem);
  }

  private Command score() {
    return scoring
        .odometryAlign(Side.FRONT, Alliance.BLUE)
        .andThen(scoring.score(ScoringHeight.LOW, Side.FRONT));
  }

  // TODO jasdfhjisauhg
  /** score cube low, intake cube, score cube low, intake cube */
  private Command twoCubesLow() {
    Pose2d scoringPose1 =
        new Pose2d(Constants.Field.SCORING_POINTS.get(1), Rotation2d.fromRadians(0));
    Pose2d intakePose1 =
        new Pose2d(Constants.Field.INTAKE_POINTS.get(1), Rotation2d.fromRadians(Math.PI));
    Pose2d scoringPose2 =
        new Pose2d(Constants.Field.SCORING_POINTS.get(2), Rotation2d.fromRadians(0));
    Pose2d intakePose2 =
        new Pose2d(Constants.Field.INTAKE_POINTS.get(2), Rotation2d.fromRadians(Math.PI));
    return scoring
        .setGamePiece(GamePiece.CUBE)
        .andThen(drive.driveToPose(scoringPose1))
        .andThen(scoring.score(ScoringHeight.LOW, Side.FRONT))
        .andThen(drive.driveToPose(scoringPose1, intakePose1))
        .andThen(autoIntake(Constants.Positions.BACK_INTAKE))
        .andThen(drive.driveToPose(intakePose1, scoringPose2))
        .andThen(scoring.score(ScoringHeight.LOW, Side.FRONT))
        .andThen(drive.driveToPose(scoringPose2, intakePose2))
        .andThen(autoIntake(Constants.Positions.BACK_INTAKE));
  }

  /** returns currently selected auto command */
  public Command get() {
    return autoChooser.getSelected();
  }
}
