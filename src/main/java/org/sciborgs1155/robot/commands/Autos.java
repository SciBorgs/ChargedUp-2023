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

  private final SwerveAutoBuilder builder;

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

    builder =
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
    return builder.fullAuto(PathPlanner.loadPathGroup(pathName, Constants.Drive.CONSTRAINTS));
  }

  /** back cone, cube intake, back cube */
  public Command twoGamepiece() {
    return followAutoPath("cone cube" + startingPosChooser.getSelected().suffix);
  }

  public Command driveToBalance() {
    return Commands.run(() -> drive.drive(0.75, 0, 0, false), drive)
        .until(() -> Math.abs(drive.getPitch()) >= 14.5);
  }

  public Command highConeScore() {
    return Commands.sequence(
        defaultOdometryReset(GamePiece.CONE, Rotation2d.fromRadians(0)),
        eventMarkers.get("initialIntake"),
        eventMarkers.get("backHighCone"),
        eventMarkers.get("score"));
  }

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

  /** no PPL */
  public Command balance() {
    return Commands.sequence(driveToBalance(), drive.balance());
  }

  /** no PPL */
  public Command cubeBalance() {
    return Commands.sequence(backHighCubeScore(), balance().alongWith(placement.toState(STOW)));
  }

  /** no PPL */
  public Command coneBalance() {
    return Commands.sequence(highConeScore(), balance().alongWith(placement.toState(STOW)));
  }

  public Command coneLeave() {
    StartingPos startingPos = startingPosChooser.getSelected();
    return followAutoPath("cone leaveComm" + startingPos.suffix);
  }

  public Command cubeLeave() {
    StartingPos startingPos = startingPosChooser.getSelected();
    return followAutoPath("cube leaveComm" + startingPos.suffix);
  }

  /** backup: no arm */
  public Command leave() {
    return followAutoPath("leaveComm" + startingPosChooser.getSelected().suffix);
  }

  /** backup: no odometry */
  public Command leaveNoOdometry() {
    return drive.drive(() -> 0.75, () -> 0, () -> 0, false).withTimeout(2.4);
  }

  /** backup: no odometry, no arm */
  public Command coneLeaveNoOdometry() {
    return this.highConeScore().andThen(leaveNoOdometry());
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

  @Override
  public void initSendable(SendableBuilder builder) {
    startingPosChooser.initSendable(builder);
  }
}
