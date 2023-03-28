package org.sciborgs1155.robot.commands;

import static org.sciborgs1155.robot.Constants.Positions.*;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Map;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Constants.*;
import org.sciborgs1155.robot.commands.Scoring.*;
import org.sciborgs1155.robot.subsystems.Drive;
import org.sciborgs1155.robot.subsystems.Intake;

public final class Autos implements Sendable {

  public enum StartingPos {
    SUBSTATION(" l"),
    CENTER(" c"),
    CORNER(" r");

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

  public Autos(Drive drive, Placement placement, Intake intake) {
    this.drive = drive;
    this.intake = intake;
    this.placement = placement;

    eventMarkers = genEventMarkers();

    startingPosChooser = new SendableChooser<StartingPos>();
    startingPosChooser.setDefaultOption("substation", StartingPos.SUBSTATION);
    startingPosChooser.addOption("corner", StartingPos.CORNER);
    startingPosChooser.addOption("center", StartingPos.CENTER);
  }

  private Map<String, Command> genEventMarkers() {
    return Map.ofEntries(
        Map.entry("backHighCone", placement.safeToState(BACK_HIGH_CONE)),
        Map.entry("backHighCube", placement.safeToState(BACK_HIGH_CUBE)),
        Map.entry("frontHighCube", placement.safeToState(FRONT_HIGH_CUBE)),
        Map.entry(
            "outtakeCone",
            intake.outtake().withTimeout(Auto.CONE_OUTTAKE_TIME).andThen(intake.stop())),
        Map.entry(
            "outtakeCube",
            intake.outtake().withTimeout(Auto.CUBE_OUTTAKE_TIME).andThen(intake.stop())),
        Map.entry("score", intake.outtake().withTimeout(3).andThen(intake.stop())),
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
        Map.entry("initialIntake", intake.intake().withTimeout(0.6).andThen(intake.stop())));
  }

  private Command followAutoPath(String pathName) {
    var builder =
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

    return builder.fullAuto(PathPlanner.loadPathGroup(pathName, Constants.Drive.CONSTRAINTS));
  }

  // public Command oneMeterTest() {
  //   ProfiledPIDController controller =
  //     new ProfiledPIDController(
  //       Constants.Drive.CARTESIAN.p(),
  //       Constants.Drive.CARTESIAN.i(),
  //       Constants.Drive.CARTESIAN.d(),
  //       new Constraints(
  //         Constants.Drive.CONSTRAINTS.maxVelocity,
  //         Constants.Drive.CONSTRAINTS.maxAcceleration));
  // }

  public Command pplOneMeterTest() {
    return followAutoPath("one meter");
  }

  public Command coneCubeEngage() {
    StartingPos startingPos = startingPosChooser.getSelected();
    // if (startingPos == StartingPos.CENTER) {
    //   DriverStation.reportError(
    //       "Error encountered when attempting to generate auto command: Cannot do cone cube engage
    // auto path from center",
    //       true);
    //   return Commands.none();
    // }
    return Commands.sequence(
        followAutoPath("cone cube balance" + startingPos.suffix), drive.balance());
  }

  public Command coneCubeIntake() {
    StartingPos startingPos = startingPosChooser.getSelected();
    // if (startingPos == StartingPos.CENTER) {
    //   DriverStation.reportError(
    //       "Error encountered when attempting to generate auto command: Cannot do cone cube intake
    // auto path from center",
    //       true);
    //   return Commands.none();
    // }
    return followAutoPath("cone cube intake" + startingPos.suffix);
  }

  public Command scoreLeaveNoPPL() {
    double offset =
        switch (DriverStation.getAlliance()) {
          case Blue -> 6;
          case Red -> -6;
          case Invalid -> 0;
        };
    return this.highConeScore()
        .andThen(
            drive.driveToPose(
                new Pose2d(
                    drive.getPose().getX() + offset,
                    drive.getPose().getY(),
                    drive.getPose().getRotation()),
                false));
  }

  public Command coneLeaveNoOdometry() {
    return this.highConeScore()
    .andThen(drive.drive(() -> 0.75, () -> 0, () -> 0, false))
    .withTimeout(3);
  }

  public Command leaveNoOdometry() {
    return drive.drive(() -> 0.75, () -> 0, () -> 0, false)
    .withTimeout(2.4);
  }

  public Command cubeBalance() {
    // if (startingPosChooser.getSelected() != StartingPos.CENTER) {
    //   DriverStation.reportError(
    //       "Error encountered when attempting to generate auto command: Cube balance path can only
    // be done from center",
    //       true);
    //   return Commands.none();
    // }
    return Commands.sequence(followAutoPath("cube balance"), drive.balance().withTimeout(3));
  }

  public Command driveToBalance() {
    return Commands.run(() -> drive.drive(0.75, 0, 0, false), drive)
        .until(() -> Math.abs(drive.getPitch()) >= 14.5);
  }

  public Command balanceNoPPL() {
    return Commands.sequence(driveToBalance(), drive.balance());
  }

  public Command scoreBalanceNoPPL() {
    return Commands.sequence(
        backHighCubeScore(), balanceNoPPL().alongWith(placement.toState(STOW)));
  }

  public Command coneBalance() {
    // if (startingPosChooser.getSelected() != StartingPos.CENTER) {
    //   DriverStation.reportError(
    //       "Error encountered when attempting to generate auto command: Cone balance path can only
    // be done from center",
    //       true);
    //   return Commands.none();
    // }
    return Commands.sequence(followAutoPath("cone balance"), drive.balance());
  }

  public Command coneLeave() {
    StartingPos startingPos = startingPosChooser.getSelected();
    // if (startingPos == StartingPos.CENTER) {
    //   DriverStation.reportError(
    //       "Error encountered when attempting to generate auto command: Cone leave path cannot be
    // done from the center",
    //       true);
    //   return Commands.none();
    // }
    return followAutoPath("cone leaveComm" + startingPos.suffix);
  }

  public Command cubeLeave() {
    StartingPos startingPos = startingPosChooser.getSelected();
    // if (startingPos == StartingPos.CENTER) {
    //   DriverStation.reportError(
    //       "Error encountered when attempting to generate auto command: Cube leave path cannot be
    // done from the center",
    //       true);
    //   return Commands.none();
    // }
    return followAutoPath("cube leaveComm" + startingPos.suffix);
  }

  public Command justBalance() {
    // if (startingPosChooser.getSelected() != StartingPos.CENTER) {
    //   DriverStation.reportError(
    //       "Error encountered when attempting to generate auto command: Just balance path can only
    // be done from center",
    //       true);
    //   return Commands.none();
    //
    return Commands.sequence(followAutoPath("balance"), drive.balance());
  }

  public Command cubeIntake() {
    return followAutoPath("cube intake" + startingPosChooser.getSelected().suffix);
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
        defaultOdometryReset(GamePiece.CONE, Rotation2d.fromRadians(0)),
        eventMarkers.get("initialIntake"),
        eventMarkers.get("backHighCone"),
        eventMarkers.get("score"));
  }

  public Command defaultOdometryReset(GamePiece gamePiece, Rotation2d rotation) {
    return Commands.runOnce(
        () ->
            drive.resetOdometry(
                new Pose2d(
                    switch (DriverStation.getAlliance()) {
                      case Blue -> 1.83;
                      case Red -> 14.67;
                      case Invalid -> -1; // should never happen!
                    },
                    switch (startingPosChooser.getSelected()) {
                      case SUBSTATION -> switch (gamePiece) {
                        case CONE -> 5.0;
                        case CUBE -> 4.42;
                      };
                      case CENTER -> switch (gamePiece) {
                        case CONE -> 3.29;
                        case CUBE -> 2.75;
                      };
                      case CORNER -> switch (gamePiece) {
                        case CONE -> 0.51;
                        case CUBE -> 1.06;
                      };
                    },
                    switch (DriverStation.getAlliance()) {
                      case Blue -> rotation;
                      case Red -> Rotation2d.fromRadians(Math.PI - rotation.getRadians());
                      case Invalid -> rotation; // should never happen!
                    })),
        drive);
  }

  // public Command highCubeScore() {
  //   return Commands.sequence(
  //       scoring.setGamePiece(GamePiece.CUBE),
  //       scoring.setSide(Side.FRONT),
  //       scoring.goTo(Level.HIGH),
  //       intake.outtake().withTimeout(2).andThen(intake.stop()));
  // }

  public Command backHighCubeScore() {
    return Commands.sequence(
        defaultOdometryReset(GamePiece.CUBE, Rotation2d.fromRadians(0)),
        eventMarkers.get("backHighCone"),
        eventMarkers.get("outtakeCube"));
  }

  public Command frontHighCubeScore() {
    return Commands.sequence(
        defaultOdometryReset(GamePiece.CUBE, Rotation2d.fromRadians(Math.PI)),
        eventMarkers.get("frontHighCube"),
        eventMarkers.get("score"));
  }

  // private Command intakeScore(Pose2d startingPos, int intakingPos, int scoringPos, GamePiece
  // gamePiece) {
  //   return
  //     drive.driveToPose(startingPos, )
  // }

  // private Pose2d intakePose(int intakePointNum, Side side) {
  //   return new Pose2d(
  //       Constants.Field.INTAKE_POINTS.get(intakePointNum), new Rotation2d(Math.PI -
  // side.rads()));
  // }

  // private Pose2d scoringPose(int scoringPointNum, Side side) {
  //   return new Pose2d(
  //       Constants.Field.SCORING_POINTS.get(scoringPointNum), new Rotation2d(side.rads()));
  // }

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
