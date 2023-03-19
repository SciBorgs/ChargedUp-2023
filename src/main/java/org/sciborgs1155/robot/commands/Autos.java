package org.sciborgs1155.robot.commands;

import static org.sciborgs1155.robot.Constants.Positions.*;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;
import org.sciborgs1155.lib.Vision;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.commands.Scoring.*;
import org.sciborgs1155.robot.subsystems.Drive;
import org.sciborgs1155.robot.subsystems.Intake;

public final class Autos implements Loggable {

  public enum StartingPos {
    LEFT(" l"),
    CENTER(" c"),
    RIGHT(" r");

    public final String suffix;

    StartingPos(String suffix) {
      this.suffix = suffix;
    }
  }

  @Log private final SendableChooser<Supplier<Command>> autoChooser;
  @Log private final SendableChooser<StartingPos> startingPosChooser;

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
            Constants.Drive.CARTESIAN.toPPL(),
            Constants.Drive.ANGULAR.toPPL(),
            drive::setModuleStates,
            eventMarkers,
            true,
            drive);

    startingPosChooser = new SendableChooser<StartingPos>();
    startingPosChooser.setDefaultOption("left", StartingPos.LEFT);
    startingPosChooser.addOption("right", StartingPos.RIGHT);
    startingPosChooser.addOption("center", StartingPos.CENTER);

    autoChooser = new SendableChooser<Supplier<Command>>();
    autoChooser.addOption("balance", this::justBalance);
    autoChooser.addOption("cone score", this::highConeScore);
    autoChooser.addOption("cube score", this::highCubeScore);
    autoChooser.addOption("cone, cube, engage", this::coneCubeEngage);
    autoChooser.addOption("cone, cube, intake", this::coneCubeIntake);
    autoChooser.addOption("cube, balance", this::cubeBalance);
    autoChooser.addOption("cone leave", this::coneLeave);
    autoChooser.addOption("cube leave", this::cubeLeave);
    autoChooser.setDefaultOption("cone/cube leave (no ppl)", this::scoreLeaveNoPPL);
  }

  private Map<String, Command> genEventMarkers() {
    return Map.ofEntries(
        Map.entry(
            "backHighCone",
            scoring
                .setGamePiece(GamePiece.CONE)
                .andThen(scoring.setSide(Side.BACK))
                .andThen(scoring.goTo(Level.HIGH))),
        Map.entry(
            "frontHighCube",
            scoring
                .setGamePiece(GamePiece.CUBE)
                .andThen(scoring.setSide(Side.FRONT))
                .andThen(scoring.goTo(Level.HIGH))),
        Map.entry("score", intake.outtake().withTimeout(0.5).andThen(intake.stop())),
        Map.entry(
            "frontIntake",
            placement
                .safeToState(Constants.Positions.FRONT_INTAKE)
                .andThen(intake.intake())
                .withTimeout(4)
                .andThen(intake.stop())),
        Map.entry(
            "backIntake",
            placement
                .safeToState(Constants.Positions.BACK_INTAKE)
                .andThen(intake.intake())
                .withTimeout(4)
                .andThen(intake.stop())),
        Map.entry("stow", placement.safeToState(STOW)));
  }

  private Command followAutoPath(String pathName, boolean resetOdometry) {
    PathPlannerTrajectory trajectory = PathPlanner.loadPath(pathName, Constants.Drive.CONSTRAINTS);
    Command reset = resetOdometry ? autoBuilder.resetPose(trajectory) : Commands.none();
    return reset.andThen(
        autoBuilder.followPathWithEvents(
            PathPlanner.loadPath(pathName, Constants.Drive.CONSTRAINTS)));
  }

  private Command coneCubeEngage() {
    StartingPos startingPos = startingPosChooser.getSelected();
    if (startingPos == StartingPos.CENTER) {
      throw new RuntimeException("cannot do cone cube engage auto path from center");
    }
    return Commands.sequence(
        intake.intake().withTimeout(1).andThen(intake.stop()),
        followAutoPath("cone cube balance" + startingPos.suffix, true),
        drive.balanceOrthogonal());
  }

  private Command coneCubeIntake() {
    StartingPos startingPos = startingPosChooser.getSelected();
    if (startingPos == StartingPos.CENTER) {
      throw new RuntimeException("cannot do cone cube intake auto path from center");
    }
    return Commands.sequence(
        intake.intake().withTimeout(0.5).andThen(intake.stop()),
        followAutoPath("cone cube intake" + startingPos.suffix, true));
  }

  private Command scoreLeaveNoPPL() {
    return this.highConeScore().andThen(
        drive.driveToPose(new Pose2d(
          drive.getPose().getX() + 6,
          drive.getPose().getY(), 
          drive.getPose().getRotation())));
  }

  private Command cubeBalance() {
    if (startingPosChooser.getSelected() != StartingPos.CENTER) {
      throw new RuntimeException("cube balance path can only be done from center");
    }
    return followAutoPath("cube balance", true).andThen(drive.balanceOrthogonal());
  }

  private Command coneLeave() {
    StartingPos startingPos = startingPosChooser.getSelected();
    if (startingPos == StartingPos.CENTER) {
      throw new RuntimeException("cone leave path cannot be done from the center");
    }
    return Commands.sequence(
        intake.intake().withTimeout(0.5).andThen(intake.stop()),
        followAutoPath("cone leaveComm" + startingPos.suffix, true));
  }

  private Command cubeLeave() {
    StartingPos startingPos = startingPosChooser.getSelected();
    if (startingPos == StartingPos.CENTER) {
      throw new RuntimeException("cube leave path cannot be done from the center");
    }
    return followAutoPath("cube leaveComm" + startingPos.suffix, true);
  }

  private Command justBalance() {
    if (startingPosChooser.getSelected() != StartingPos.CENTER) {
      throw new RuntimeException("just balance path can only be done from center");
    }
    return followAutoPath("balance", true).andThen(drive.balanceOrthogonal());
  }

  private Command highConeScore() {
    return Commands.sequence(
        intake.intake().withTimeout(0.5).andThen(intake.stop()),
        scoring.setGamePiece(GamePiece.CONE),
        scoring.setSide(Side.BACK),
        scoring.goTo(Level.HIGH),
        intake.outtake().withTimeout(0.3).andThen(intake.stop()));
  }

  private Command highCubeScore() {
    return Commands.sequence(
        scoring.setGamePiece(GamePiece.CUBE),
        scoring.setSide(Side.FRONT),
        scoring.goTo(Level.HIGH),
        intake.outtake().withTimeout(2).andThen(intake.stop()));
  }

  private Command simpleDrive() {
    Pose2d end = new Pose2d(0, 5, Rotation2d.fromDegrees(0));
    return drive
        .driveToPose(end)
        .andThen(drive.driveToPose(end, new Pose2d(0, 0, Rotation2d.fromDegrees(0))));
  }

  private Command simplestDrive() {
    drive.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
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

  // private Command intakeScore(Pose2d startPose, Pose2d intakePose, ScoringState scoringState) {
  //   return drive
  //       .driveToPose(startPose, intakePose)
  //       .andThen(autoIntake())
  //       .andThen(drive.driveToPose(intakePose, scoringState.pose()))
  //       .andThen(scoring.goTo(scoringState));
  // }

  /** returns currently selected auto command */
  public Command get() {
    if (autoChooser.getSelected() == null) {
      throw new RuntimeException("no starting position selected!");
    }
    return autoChooser.getSelected().get();
  }
}
