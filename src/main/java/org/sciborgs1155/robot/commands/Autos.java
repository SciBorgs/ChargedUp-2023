package org.sciborgs1155.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import java.util.List;
import java.util.Map;
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

  public final Map<String, Command> eventMarkers;
  public final SwerveAutoBuilder autoBuilder;

  public Autos(Drive drive, Placement placement, Vision vision, Intake intake, Scoring scoring) {
    this.drive = drive;
    this.vision = vision;
    this.intake = intake;
    this.placement = placement;
    this.scoring = scoring;

    eventMarkers = genEventMarkers();

    this.autoBuilder =
        new SwerveAutoBuilder(
            drive::getPose,
            drive::resetOdometry,
            drive.kinematics,
            new com.pathplanner.lib.auto.PIDConstants(5.0, 0.0, 0.0),
            new com.pathplanner.lib.auto.PIDConstants(0.5, 0.0, 0.0),
            drive::setModuleStates,
            eventMarkers,
            true,
            drive);
    // commands

    autoChooser = new SendableChooser<Command>();
    autoChooser.setDefaultOption("simplest drive", simplestDrive());
    autoChooser.addOption("simple drive", simpleDrive());
    autoChooser.addOption("meandering drive", meanderingDrive());
    autoChooser.addOption("balance", balance());
    autoChooser.addOption("goofy", goofy());
    autoChooser.addOption("goofyApp", goofyApp());
    autoChooser.addOption("score", highConeScore());
    autoChooser.addOption("align score", allignScore());
    autoChooser.addOption("intake", autoIntake(Constants.Positions.FRONT_INTAKE));
    autoChooser.addOption("cone, cube, engage", coneCubeEngage());
  }

  private Map<String, Command> genEventMarkers() {
    return Map.ofEntries(
        Map.entry(
            "backHighCone",
            scoring
                .setGamePiece(GamePiece.CONE)
                .andThen(scoring.score(ScoringHeight.HIGH, Side.BACK))
                .andThen(placement.safeToState(Constants.Positions.STOW))),
        Map.entry(
            "frontHighCube",
            scoring
                .setGamePiece(GamePiece.CUBE)
                .andThen(scoring.score(ScoringHeight.HIGH, Side.FRONT))
                .andThen(placement.safeToState(Constants.Positions.STOW))),
        Map.entry(
            "backHighCube",
            scoring
                .setGamePiece(GamePiece.CUBE)
                .andThen(scoring.score(ScoringHeight.HIGH, Side.BACK))
                .andThen(placement.safeToState(Constants.Positions.STOW))));
  }

  private Command followAutoPath(String pathName) {
    return autoBuilder.followPathWithEvents(
        PathPlanner.loadPath(pathName, Constants.Drive.CONSTRAINTS));
  }

  private Command coneCubeEngage() {
    return followAutoPath("cone score to intake")
        .andThen(autoIntake(Constants.Positions.FRONT_INTAKE))
        .andThen(followAutoPath("intake to cube score to balance"))
        .andThen(balance());
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

  // private Command intakeScore(Pose2d startingPos, int intakingPos, int scoringPos, GamePiece
  // gamePiece) {
  //   return
  //     drive.driveToPose(startingPos, )
  // }

  private Pose2d intakePose(int intakePointNum, Side side) {
    return new Pose2d(
        Constants.Field.INTAKE_POINTS.get(intakePointNum), new Rotation2d(Math.PI - side.rads()));
  }

  private Pose2d scoringPose(int scoringPointNum, Side side) {
    return new Pose2d(
        Constants.Field.SCORING_POINTS.get(scoringPointNum), new Rotation2d(side.rads()));
  }

  record ScoringState(Pose2d pose, ScoringHeight height, Side side) {}

  record IntakeState(Pose2d pose, PlacementState state) {}

  private Command intakeScore(
      Pose2d startPose, IntakeState intakeState, ScoringState scoringState) {
    return drive
        .driveToPose(startPose, intakeState.pose)
        .andThen(autoIntake(intakeState.state))
        .andThen(drive.driveToPose(intakeState.pose, scoringState.pose))
        .andThen(scoring.score(scoringState.height, scoringState.side));
  }

  /** returns currently selected auto command */
  public Command get() {
    return autoChooser.getSelected();
  }
}
