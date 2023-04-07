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

    eventMarkers =
        Map.ofEntries(
            Map.entry("backHighCone", placement.goTo(BACK_HIGH_CONE, false)),
            Map.entry("backHighCube", placement.goTo(BACK_HIGH_CUBE, false)),
            Map.entry("frontHighCube", placement.goTo(FRONT_HIGH_CUBE, false)),
            Map.entry("outtakeCone", outtake(GamePiece.CONE)),
            Map.entry("outtakeCube", outtake(GamePiece.CUBE)),
            Map.entry("frontIntake", frontMovingIntake()),
            Map.entry("stow", placement.goTo(STOW, false)),
            Map.entry("balanceState", placement.goTo(BALANCE, false)),
            Map.entry("initialIntake", initialIntake()));

    startingPosChooser = new SendableChooser<StartingPos>();
    startingPosChooser.setDefaultOption("substation", StartingPos.SUBSTATION);
    startingPosChooser.addOption("corner", StartingPos.CORNER);
    startingPosChooser.addOption("center", StartingPos.CENTER);

    builder =
        new SwerveAutoBuilder(
            drive::getPose,
            drive::resetOdometry,
            drive.kinematics,
            Constants.Drive.TRANSLATION.toPPL(),
            Constants.Drive.ROTATION.toPPL(),
            drive::setModuleStates,
            eventMarkers,
            true,
            drive);
  }

  private Command outtake(GamePiece gamePiece) {
    return Commands.sequence(
        intake
            .outtake()
            .withTimeout(
                switch (gamePiece) {
                  case CONE -> Auto.CONE_OUTTAKE_TIME;
                  case CUBE -> Auto.CUBE_OUTTAKE_TIME;
                }),
        intake.stop());
  }

  private Command frontMovingIntake() {
    return Commands.sequence(
        placement.goTo(Constants.Positions.FRONT_INTAKE, false),
        intake.intake().withTimeout(Auto.MOVING_INTAKE_TIME),
        intake.stop());
  }

  private Command initialIntake() {
    return Commands.sequence(intake.intake().withTimeout(Auto.INITIAL_INTAKE_TIME), intake.stop());
  }

  private Command followAutoPath(String pathName) {
    return builder.fullAuto(PathPlanner.loadPathGroup(pathName, Constants.Drive.CONSTRAINTS));
  }

  /** back cone, cube intake, back cube */
  public Command twoGamepiece() {
    return followAutoPath("cone cube" + startingPosChooser.getSelected().suffix);
  }

  public Command fullBalance() {
    var trajectory = PathPlanner.loadPath("balance", Constants.Drive.CONSTRAINTS);
    // var initialState =
    //               PathPlannerTrajectory.transformStateForAlliance(
    //                   trajectory.getInitialState(), DriverStation.getAlliance());
    // Pose2d initialPose = new Pose2d(
    //   initialState.poseMeters.getTranslation(), initialState.holonomicRotation);

    // return Commands.runOnce(() -> drive.resetOdometry(initialPose), drive)
    //     .andThen(drive.follow(trajectory, false, true))
    return drive
        .follow("balance", false, true)
        // followAutoPath("balance")
        .andThen(drive.balance())
        .withTimeout(10)
        .andThen(Commands.print("balanced!"))
        .andThen(Commands.run(() -> drive.drive(-0.3, 0, 0, false), drive).withTimeout(0.1))
        .andThen(drive.lock())
        .andThen(Commands.print("locked!"))
        .withName("balance auto");
    // .andThen(drive.balance())
    // .andThen(Commands.run(() -> drive.drive(-0.60, 0, 0, false), drive).withTimeout(0.1))
    // .andThen(drive.lock())
    // .withName("balance auto").withTimeout(8);
  }

  public Command highConeScore() {
    return Commands.sequence(
        defaultOdometryReset(GamePiece.CONE, Rotation2d.fromRadians(0)),
        initialIntake(),
        placement.goTo(BACK_HIGH_CONE, false).withTimeout(5),
        outtake(GamePiece.CONE));
  }

  public Command backHighCubeScore() {
    return Commands.sequence(
        defaultOdometryReset(GamePiece.CUBE, Rotation2d.fromRadians(0)),
        placement.goTo(BACK_HIGH_CUBE, false).withTimeout(5),
        // Commands.print("going to state..."),
        // placement.goTo(FRONT_HIGH_CUBE, false),
        // Commands.print("got to state!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"),
        outtake(GamePiece.CUBE));
  }

  public Command frontHighCubeScore() {
    return Commands.sequence(
        defaultOdometryReset(GamePiece.CUBE, Rotation2d.fromRadians(Math.PI)),
        placement.goTo(FRONT_HIGH_CUBE, false).withTimeout(5),
        outtake(GamePiece.CUBE));
  }

  /** no PPL */
  public Command cubeBalance() {
    return Commands.sequence(
        backHighCubeScore(), placement.goTo(BALANCE, false).withTimeout(2.5), fullBalance());
  }

  /** no PPL */
  public Command coneBalance() {
    return Commands.sequence(
        highConeScore(), fullBalance().alongWith(placement.goTo(BALANCE, false)));
  }

  public Command coneLeave() {
    return followAutoPath("cone leaveComm" + startingPosChooser.getSelected().suffix);
  }

  public Command cubeLeave() {
    return followAutoPath("cube leaveComm" + startingPosChooser.getSelected().suffix);
  }

  public Command lowCubeLeave() {
    return Commands.sequence(
        placement.goTo(FRONT_INTAKE, false),
        outtake(GamePiece.CUBE),
        followAutoPath("leaveComm l backwards"));
  }

  /** backup: no arm */
  public Command leave() {
    StartingPos startingPos = startingPosChooser.getSelected();
    if (startingPos == StartingPos.CENTER) {
      return leaveNoOdometry();
    }
    return followAutoPath("leaveComm" + startingPosChooser.getSelected().suffix);
  }

  /** backup: no odometry */
  public Command leaveNoOdometry() {
    return drive.drive(() -> 0.75, () -> 0, () -> 0, false).withTimeout(2.4);
  }

  /** backup: no odometry, no arm */
  public Command coneLeaveNoOdometry() {
    return Commands.sequence(
        highConeScore(), leaveNoOdometry().alongWith(placement.goTo(STOW, false)));
  }

  /** backup: no odometry, no arm */
  public Command cubeLeaveNoOdometry() {
    return Commands.sequence(
        backHighCubeScore(), leaveNoOdometry().alongWith(placement.goTo(STOW, false)));
  }

  public Command scoreOneMeterTest() {
    return Commands.sequence(backHighCubeScore(), followAutoPath("one meter"));
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

  public Command betterDefaultOdometryReset(GamePiece gamePiece, Rotation2d rotation) {
    return Commands.runOnce(
        () ->
            drive.resetOdometry(
                new Pose2d(
                    switch (DriverStation.getAlliance()) {
                      case Blue -> 1.83;
                      case Red -> 14.67;
                      case Invalid -> -1; // should never happen!
                    },
                    switch (gamePiece) {
                      case CONE -> Constants.Field.SCORING_POINTS_CONE
                          .get(
                              switch (startingPosChooser.getSelected()) {
                                case SUBSTATION -> 1;
                                case CENTER -> 3;
                                case CORNER -> 6;
                              })
                          .getY();
                      case CUBE -> Constants.Field.SCORING_POINTS_CUBE
                          .get(
                              switch (startingPosChooser.getSelected()) {
                                case SUBSTATION -> 1;
                                case CENTER -> 2;
                                case CORNER -> 3;
                              })
                          .getY();
                    },
                    switch (DriverStation.getAlliance()) {
                      case Blue -> rotation;
                      case Red -> Rotation2d.fromRadians(Math.PI - rotation.getRadians());
                      case Invalid -> rotation;
                    } // should never happen!
                    )),
        drive);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    startingPosChooser.initSendable(builder);
  }
}
