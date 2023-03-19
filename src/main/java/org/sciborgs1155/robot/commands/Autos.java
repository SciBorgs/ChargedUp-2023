package org.sciborgs1155.robot.commands;

import static org.sciborgs1155.robot.Constants.Positions.*;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Map;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.commands.Scoring.*;
import org.sciborgs1155.robot.subsystems.Drive;
import org.sciborgs1155.robot.subsystems.Intake;

public final class Autos implements Sendable {

  public enum StartingPos {
    LEFT(" l"),
    CENTER(" c"),
    RIGHT(" r");

    public final String suffix;

    StartingPos(String suffix) {
      this.suffix = suffix;
    }
  }

  private final Drive drive;
  private final Placement placement;
  private final Intake intake;

  private final Map<String, Command> eventMarkers;
  private final SendableChooser<StartingPos> startingPosChooser;

  private final SwerveAutoBuilder autoBuilder;

  public Autos(Drive drive, Placement placement, Intake intake) {
    this.drive = drive;
    this.intake = intake;
    this.placement = placement;

    eventMarkers = genEventMarkers();

    autoBuilder =
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

    startingPosChooser = new SendableChooser<>();
    startingPosChooser.setDefaultOption("left", StartingPos.LEFT);
    startingPosChooser.addOption("right", StartingPos.RIGHT);
    startingPosChooser.addOption("center", StartingPos.CENTER);
  }

  private Map<String, Command> genEventMarkers() {
    return Map.ofEntries(
        Map.entry("backHighCone", placement.safeToState(BACK_HIGH_CONE)),
        Map.entry("frontHighCube", placement.safeToState(FRONT_HIGH_CUBE)),
        Map.entry("score", intake.outtake().withTimeout(1).andThen(intake.stop())),
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
        Map.entry("stow", placement.safeToState(STOW)),
        Map.entry("initialIntake", intake.intake().withTimeout(0.5).andThen(intake.stop())));
  }

  public Command followAutoPath(String pathName, boolean resetOdometry) {
    PathPlannerTrajectory trajectory = PathPlanner.loadPath(pathName, Constants.Drive.CONSTRAINTS);
    Command reset = resetOdometry ? autoBuilder.resetPose(trajectory) : Commands.none();
    return reset.andThen(autoBuilder.followPathWithEvents(trajectory));
  }

  public Command coneCubeEngage() {
    StartingPos startingPos = startingPosChooser.getSelected();
    if (startingPos == StartingPos.CENTER) {
      throw new RuntimeException("cannot do cone cube engage auto path from center");
    }
    return Commands.sequence(
        highConeScore(),
        followAutoPath("cone cube balance" + startingPos.suffix, true),
        drive.balanceOrthogonal());
  }

  public Command coneCubeIntake() {
    StartingPos startingPos = startingPosChooser.getSelected();
    if (startingPos == StartingPos.CENTER) {
      throw new RuntimeException("cannot do cone cube intake auto path from center");
    }
    return Commands.sequence(
        highConeScore(), followAutoPath("cone cube intake" + startingPos.suffix, true));
  }

  public Command scoreLeaveNoPPL() {
    return this.highConeScore()
        .andThen(
            drive.driveToPose(
                new Pose2d(
                    drive.getPose().getX() + 6,
                    drive.getPose().getY(),
                    drive.getPose().getRotation())));
  }

  public Command cubeBalance() {
    if (startingPosChooser.getSelected() != StartingPos.CENTER) {
      throw new RuntimeException("cube balance path can only be done from center");
    }
    return Commands.sequence(
        frontHighCubeScore(),
        eventMarkers.get("stow"),
        followAutoPath("cube balance", true),
        drive.balanceOrthogonal());
  }

  public Command coneLeave() {
    StartingPos startingPos = startingPosChooser.getSelected();
    if (startingPos == StartingPos.CENTER) {
      throw new RuntimeException("cone leave path cannot be done from the center");
    }
    return Commands.sequence(
        highConeScore(), followAutoPath("cone leaveComm" + startingPos.suffix, true));
  }

  public Command cubeLeave() {
    StartingPos startingPos = startingPosChooser.getSelected();
    if (startingPos == StartingPos.CENTER) {
      throw new RuntimeException("cube leave path cannot be done from the center");
    }
    return Commands.sequence(
        frontHighCubeScore(), followAutoPath("cube leaveComm" + startingPos.suffix, true));
  }

  public Command justBalance() {
    if (startingPosChooser.getSelected() != StartingPos.CENTER) {
      throw new RuntimeException("just balance path can only be done from center");
    }
    return Commands.sequence(
        eventMarkers.get("stow"), followAutoPath("balance", true), drive.balanceOrthogonal());
  }

  // public Command highConeScore() {
  //   return Commands.sequence(
  //       intake.intake().withTimeout(0.5).andThen(intake.stop()),
  //       scoring.setGamePiece(GamePiece.CONE),
  //       scoring.setSide(Side.BACK),
  //       scoring.goTo(Level.HIGH),
  //       intake.outtake().withTimeout(0.3).andThen(intake.stop()));
  // }

  public Command highConeScore() {
    return Commands.sequence(
        eventMarkers.get("initialIntake"),
        eventMarkers.get("backHighCone"),
        eventMarkers.get("score"));
  }

  // public Command highCubeScore() {
  //   return Commands.sequence(
  //       scoring.setGamePiece(GamePiece.CUBE),
  //       scoring.setSide(Side.FRONT),
  //       scoring.goTo(Level.HIGH),
  //       intake.outtake().withTimeout(2).andThen(intake.stop()));
  // }

  public Command backHighCubeScore() {
    return Commands.sequence(eventMarkers.get("backHighCone"), eventMarkers.get("score"));
  }

  public Command frontHighCubeScore() {
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

  // public Command intakeScore(Pose2d startPose, Pose2d intakePose, ScoringState scoringState) {
  //   return drive
  //       .driveToPose(startPose, intakePose)
  //       .andThen(autoIntake())
  //       .andThen(drive.driveToPose(intakePose, scoringState.pose()))
  //       .andThen(scoring.goTo(scoringState));
  // }

  @Override
  public void initSendable(SendableBuilder builder) {
    startingPosChooser.initSendable(builder);
  }
}
