package org.sciborgs1155.robot.commands;

import static org.sciborgs1155.robot.Constants.Positions.*;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
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
    autoChooser.addOption("back cube score", this::backHighCubeScore);
    autoChooser.addOption("front cube score", this::frontHighCubeScore);
    autoChooser.addOption("cone, cube, engage", this::coneCubeEngage);
    autoChooser.addOption("cone, cube, intake", this::coneCubeIntake);
    autoChooser.addOption("cube, balance", this::cubeBalance);
    autoChooser.addOption("cone leave", this::coneLeave);
    autoChooser.addOption("cube leave", this::cubeLeave);
    autoChooser.addOption("cone/cube leave (no ppl)", this::scoreLeaveNoPPL);
  }

  private Map<String, Command> genEventMarkers() {
    return Map.ofEntries(
        Map.entry(
            "backHighCone",
            Commands.sequence(
                scoring.setGamePiece(GamePiece.CONE),
                scoring.setSide(Side.BACK),
                scoring.goTo(Level.HIGH))),
        Map.entry(
            "frontHighCube",
            Commands.sequence(
                scoring.setGamePiece(GamePiece.CUBE),
                scoring.setSide(Side.FRONT),
                scoring.goTo(Level.HIGH))),
        Map.entry("score", intake.outtake().withTimeout(0.3).andThen(intake.stop())),
        Map.entry(
            "frontIntake",
            Commands.sequence(
                placement.safeToState(Constants.Positions.FRONT_INTAKE),
                intake.intake().withTimeout(4),
                intake.stop())),
        Map.entry(
            "backIntake",
            Commands.sequence(
                placement.safeToState(Constants.Positions.BACK_INTAKE),
                intake.intake().withTimeout(4),
                intake.stop())),
        Map.entry("stow", placement.safeToState(STOW)),
        Map.entry("initialIntake", intake.intake().withTimeout(0.5).andThen(intake.stop())));
  }

  private Command followAutoPath(String pathName) {
    return autoBuilder.fullAuto(PathPlanner.loadPathGroup(pathName, Constants.Drive.CONSTRAINTS));
  }

  private Command coneCubeEngage() {
    StartingPos startingPos = startingPosChooser.getSelected();
    if (startingPos == StartingPos.CENTER) {
      throw new RuntimeException("cannot do cone cube engage auto path from center");
    }
    return Commands.sequence(
        followAutoPath("cone cube balance" + startingPos.suffix), drive.balanceOrthogonal());
  }

  private Command coneCubeIntake() {
    StartingPos startingPos = startingPosChooser.getSelected();
    if (startingPos == StartingPos.CENTER) {
      throw new RuntimeException("cannot do cone cube intake auto path from center");
    }
    return followAutoPath("cone cube intake" + startingPos.suffix);
  }

  private Command scoreLeaveNoPPL() {
    return this.highConeScore()
        .andThen(
            drive.driveToPose(
                new Pose2d(
                    drive.getPose().getX() + 6,
                    drive.getPose().getY(),
                    drive.getPose().getRotation())));
  }

  private Command cubeBalance() {
    if (startingPosChooser.getSelected() != StartingPos.CENTER) {
      throw new RuntimeException("cube balance path can only be done from center");
    }
    return Commands.sequence(followAutoPath("cube balance"), drive.balanceOrthogonal());
  }

  private Command coneLeave() {
    StartingPos startingPos = startingPosChooser.getSelected();
    if (startingPos == StartingPos.CENTER) {
      throw new RuntimeException("cone leave path cannot be done from the center");
    }
    return followAutoPath("cone leaveComm" + startingPos.suffix);
  }

  private Command cubeLeave() {
    StartingPos startingPos = startingPosChooser.getSelected();
    if (startingPos == StartingPos.CENTER) {
      throw new RuntimeException("cube leave path cannot be done from the center");
    }
    return followAutoPath("cube leaveComm" + startingPos.suffix);
  }

  private Command justBalance() {
    if (startingPosChooser.getSelected() != StartingPos.CENTER) {
      throw new RuntimeException("just balance path can only be done from center");
    }
    return Commands.sequence(followAutoPath("balance"), drive.balanceOrthogonal());
  }

  // private Command highConeScore() {
  //   return Commands.sequence(
  //       intake.intake().withTimeout(0.5).andThen(intake.stop()),
  //       scoring.setGamePiece(GamePiece.CONE),
  //       scoring.setSide(Side.BACK),
  //       scoring.goTo(Level.HIGH),
  //       intake.outtake().withTimeout(0.3).andThen(intake.stop()));
  // }

  private Command highConeScore() {
    return Commands.sequence(
        eventMarkers.get("initialIntake"),
        eventMarkers.get("backHighCone"),
        eventMarkers.get("score"));
  }

  // private Command highCubeScore() {
  //   return Commands.sequence(
  //       scoring.setGamePiece(GamePiece.CUBE),
  //       scoring.setSide(Side.FRONT),
  //       scoring.goTo(Level.HIGH),
  //       intake.outtake().withTimeout(2).andThen(intake.stop()));
  // }

  private Command backHighCubeScore() {
    return Commands.sequence(eventMarkers.get("backHighCone"), eventMarkers.get("score"));
  }

  private Command frontHighCubeScore() {
    return Commands.sequence(eventMarkers.get("frontHighCube"), eventMarkers.get("score"));
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
