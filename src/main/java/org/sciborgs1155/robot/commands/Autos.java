package org.sciborgs1155.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import java.util.List;
import org.sciborgs1155.lib.PlacementState;
import org.sciborgs1155.lib.Vision;
import org.sciborgs1155.robot.Constants.*;
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
    autoChooser.setDefaultOption("simplest drive", simplestDrive());
    autoChooser.addOption("simple drive", simpleDrive());
    autoChooser.addOption("meandering drive", meanderingDrive());
    autoChooser.addOption("balance", balance());
    autoChooser.addOption("goofy", goofy());
    autoChooser.addOption("goofyApp", goofyApp());
    autoChooser.addOption("2 cubes low", twoCubesLow());
    autoChooser.addOption("2 cubes engage", twoCubesEngage());
    autoChooser.addOption("score", highConeScore());
    autoChooser.addOption("align score", allignScore());
    autoChooser.addOption("intake", autoIntake(Positions.FRONT_INTAKE));
  }

  private Command simpleDrive() {
    Pose2d end = new Pose2d(0, 5, Rotation2d.fromDegrees(0));
    return drive
        .driveToPose(end)
        .andThen(drive.driveToPose(end, new Pose2d(0, 0, Rotation2d.fromDegrees(0))));
  }

  private Command simplestDrive() {
    return drive.driveToPose(new Pose2d(0, 5, Rotation2d.fromDegrees(0)));
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
    return drive
        .drive(() -> 0.5, () -> 0, () -> 0, true)
        .withTimeout(0.6)
        .andThen(drive.balanceOrthogonal());
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

  private Command allignScore() {
    return scoring
        .setGamePiece(GamePiece.CONE)
        .andThen(scoring.odometryAlign(Side.BACK, Alliance.BLUE))
        .andThen(scoring.score(ScoringHeight.HIGH, Side.BACK));
  }

  private Command highConeScore() {
    return scoring
        .setGamePiece(GamePiece.CONE)
        .andThen(scoring.score(ScoringHeight.HIGH, Side.BACK));
  }

  // TODO jasdfhjisauhg
  /** score cube low, intake cube, score cube low, intake cube */
  private Command twoCubesLow() {
    Pose2d startingPose = new Pose2d(Field.SCORING_POINTS.get(1), Rotation2d.fromRadians(0));
    Pose2d intakePose1 = new Pose2d(Field.INTAKE_POINTS.get(1), Rotation2d.fromRadians(Math.PI));
    Pose2d scoringPose2 = new Pose2d(Field.SCORING_POINTS.get(2), Rotation2d.fromRadians(0));
    Pose2d intakePose2 = new Pose2d(Field.INTAKE_POINTS.get(2), Rotation2d.fromRadians(Math.PI));
    return scoring
        .setGamePiece(GamePiece.CUBE)
        .andThen(scoring.score(ScoringHeight.LOW, Side.FRONT))
        .andThen(drive.driveToPose(startingPose, intakePose1))
        .andThen(autoIntake(Positions.BACK_INTAKE))
        .andThen(drive.driveToPose(intakePose1, scoringPose2))
        .andThen(scoring.score(ScoringHeight.LOW, Side.FRONT))
        .andThen(drive.driveToPose(scoringPose2, intakePose2))
        .andThen(autoIntake(Positions.BACK_INTAKE));
  }

  // TODO make accurate
  private Command twoCubesEngage() {
    Pose2d startPose = new Pose2d(Field.SCORING_POINTS.get(1), Rotation2d.fromRadians(Math.PI));
    Pose2d intakePose = new Pose2d(Field.INTAKE_POINTS.get(1), Rotation2d.fromRadians(Math.PI));
    Pose2d scorePose = new Pose2d(Field.INTAKE_POINTS.get(2), Rotation2d.fromRadians(Math.PI));
    Pose2d balancePose = new Pose2d(Field.BALANCE_POINTS.get(1), Rotation2d.fromRadians(Math.PI));
    return scoring
        .setGamePiece(GamePiece.CUBE)
        .andThen(scoring.score(ScoringHeight.HIGH, Side.BACK))
        .andThen(drive.driveToPose(startPose, intakePose))
        .andThen(autoIntake(Positions.FRONT_INTAKE))
        .andThen(drive.driveToPose(intakePose, scorePose))
        .andThen(scoring.score(ScoringHeight.HIGH, Side.BACK))
        .andThen(drive.driveToPose(scorePose, balancePose))
        .andThen(balance());
  }

  /** returns currently selected auto command */
  public Command get() {
    return autoChooser.getSelected();
  }
}
