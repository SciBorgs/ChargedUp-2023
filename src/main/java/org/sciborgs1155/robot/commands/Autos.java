package org.sciborgs1155.robot.commands;

import static org.sciborgs1155.robot.Constants.Auto.*;
import static org.sciborgs1155.robot.Constants.Field.*;
import static org.sciborgs1155.robot.commands.Autos.StartingPos.*;
import static org.sciborgs1155.robot.subsystems.Drive.loadPath;
import static org.sciborgs1155.robot.subsystems.arm.ArmState.*;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.List;
import java.util.function.Supplier;
import org.sciborgs1155.robot.subsystems.Arm;
import org.sciborgs1155.robot.subsystems.Drive;
import org.sciborgs1155.robot.subsystems.Intake;
import org.sciborgs1155.robot.subsystems.arm.ArmState;

public final class Autos implements Sendable {

  public enum StartingPos {
    BUMP,
    FLAT,
    CENTER
  }

  public static final class Paths {
    public static final List<PathPlannerTrajectory> TWO_GAMEPIECE_BUMP = loadPath("cone cube b");
    public static final List<PathPlannerTrajectory> TWO_GAMEPIECE_FLAT = loadPath("cone cube f");

    public static final List<PathPlannerTrajectory> CONE_LEAVE_BUMP = loadPath("cone leaveComm b");
    public static final List<PathPlannerTrajectory> CONE_LEAVE_FLAT = loadPath("cone leaveComm f");

    public static final List<PathPlannerTrajectory> CUBE_LEAVE_BUMP = loadPath("cube leaveComm b");
    public static final List<PathPlannerTrajectory> CUBE_LEAVE_FLAT = loadPath("cube leaveComm f");

    public static final List<PathPlannerTrajectory> CUBE_INTAKE_FLAT = loadPath("cube intake f");

    public static final List<PathPlannerTrajectory> LEAVE_BUMP = loadPath("leaveComm b");
    public static final List<PathPlannerTrajectory> LEAVE_FLAT = loadPath("leaveComm f");

    public static final List<PathPlannerTrajectory> LEAVE_FLAT_BACKWARDS =
        loadPath("leaveComm f backwards");

    public static final List<PathPlannerTrajectory> ONE_METER_TEST = loadPath("one meter");

    public static final List<PathPlannerTrajectory> BALANCE = loadPath("balance");
  }

  private final Drive drive;
  private final Arm arm;
  private final Intake intake;

  private final SendableChooser<Supplier<Command>> autoChooser;

  public Autos(Drive drive, Arm arm, Intake intake) {
    this.drive = drive;
    this.arm = arm;
    this.intake = intake;

    autoChooser = new SendableChooser<Supplier<Command>>();
    configureMainAutos();
  }

  private void configureMainAutos() {
    /* 2 gamepiece setup instructions:
     * gamepiece: cone
     * orientation: away from grid
     * starting position: cone scoring, flat side
     */
    autoChooser.addOption("2 gamepiece (flat)", () -> twoGamepiece(FLAT));

    /* cube balance setup instructions:
     * gamepiece: cube
     * orientation: away from grid
     * starting location: cube scoring, center
     */
    autoChooser.addOption("cube, balance", () -> scoreBalance(GamePiece.CUBE));

    /* cube intake setup instructions:
     * gamepiece: cube
     * orientation: away from grid
     * starting location: cube scoring, flat side
     */
    autoChooser.addOption("cube, intake (flat)", this::cubeScoreIntake);

    // backups

    /* cube score setup instructions:
     * gamepiece: cube
     * orientation: away from grid
     * starting location: cube scoring (preferably flat)
     */
    autoChooser.setDefaultOption(
        "cube score (no drive)", () -> justScore(GamePiece.CUBE, Side.FRONT, FLAT));
    /* cone score setup instructions:
     * gamepiece: cone
     * orientation: away from grid
     * set starting pos: no
     * starting location: cone scoring (preferably flat)
     */
    autoChooser.addOption(
        "cone score (no drive)", () -> justScore(GamePiece.CONE, Side.BACK, FLAT));

    // ultimate backup
    autoChooser.addOption("none", Commands::none);
  }

  private void configureExtraAutos() {
    /* balance setup instructions:
     * gamepiece: none
     * orientation: away from grid
     * starting location: in front of charge station, preferably cube scoring
     */
    autoChooser.addOption("balance", this::justBalance);

    /* cone balance setup instructions:
     * gamepiece: cone
     * orientation: away from grid
     * starting location: cone scoring, off-center (preferably left of center)
     */
    autoChooser.addOption("cone, balance", () -> scoreBalance(GamePiece.CONE));

    // simple scoring

    /* cone leave setup instructions:
     * gamepiece: cone
     * orientation: away from grid
     * starting location: cone scoring, flat side
     */
    autoChooser.addOption("cone, leave (flat)", () -> coneLeave(FLAT));

    /* cone leave setup instructions:
     * gamepiece: cone
     * orientation: away from grid
     * starting location: cone scoring, bump side
     */
    autoChooser.addOption("cone, leave (bump)", () -> coneLeave(BUMP));

    /* cube leave setup instructions:
     * gamepiece: cube
     * orientation: away from grid
     * starting location: cube scoring, flat side
     */
    autoChooser.addOption("cube, leave (flat)", () -> cubeLeave(FLAT));

    /* cube leave setup instructions:
     * gamepiece: cube
     * orientation: away from grid
     * starting location: cube scoring, bump side
     */
    autoChooser.addOption("cube, leave (bump)", () -> cubeLeave(BUMP));

    // backups

    /* leave setup instructions:
     * gamepiece: none
     * orientation: away from grid
     * set starting pos: yes
     * starting location: against grid, to one side (it should have a clear path straight forward)
     */
    autoChooser.addOption("flat leave (no arm)", () -> leave(FLAT));

    /* leave setup instructions:
     * gamepiece: none
     * orientation: away from grid
     * set starting pos: yes
     * starting location: against grid, to one side (it should have a clear path straight forward)
     */
    autoChooser.addOption("bump leave (no arm)", () -> leave(BUMP));

    autoChooser.addOption("low cube, leave (flat)", this::lowCubeLeave);
  }

  private void configureTestAutos() {
    autoChooser.addOption(
        "one meter test", () -> drive.followPath(Paths.ONE_METER_TEST.get(0), true));
    autoChooser.addOption("2 gamepeice (bump)", () -> twoGamepiece(BUMP));
  }

  private void configureAllAutos() {
    configureMainAutos();
    configureExtraAutos();
    configureTestAutos();
  }

  private Command frontMovingIntake(GamePiece gamePiece) {
    return arm.goTo(Goal.LOW, () -> gamePiece)
        .andThen(intake.intake(() -> gamePiece).withTimeout(MOVING_INTAKE_TIME));
  }

  /** back cone, cube intake, back cube */
  public Command twoGamepiece(StartingPos startingPos) {
    var pathGroup =
        switch (startingPos) {
          case BUMP -> Paths.TWO_GAMEPIECE_BUMP;
          case FLAT -> Paths.TWO_GAMEPIECE_FLAT;
          case CENTER -> Paths.TWO_GAMEPIECE_FLAT;
        };
    return Commands.sequence(
        staticOdometryReset(GamePiece.CONE, Rotation2d.fromRadians(0), startingPos),
        score(Goal.HIGH, GamePiece.CONE),
        Commands.parallel(
            Commands.sequence(Commands.waitSeconds(0.5), drive.followPath(pathGroup.get(0), false)),
            frontMovingIntake(GamePiece.CUBE)),
        Commands.parallel(drive.followPath(pathGroup.get(1), false), arm.goTo(ArmState::stow)),
        score(Goal.HIGH, GamePiece.CUBE), // TODO doesn't exist
        arm.goTo(ArmState::stow));
  }

  private Command justBalance() {
    return Commands.sequence(
        staticOdometryReset(GamePiece.CUBE, Rotation2d.fromRadians(0), CENTER), fullBalance());
  }

  public Command balance() {
    var controller = new PIDController(BALANCE.p(), BALANCE.i(), BALANCE.d());
    controller.setTolerance(PITCH_TOLERANCE);
    controller.setSetpoint(0);

    return Commands.run(
            () -> drive.drive(new ChassisSpeeds(controller.calculate(drive.getPitch()), 0, 0)),
            drive)
        .until(controller::atSetpoint)
        .andThen(drive.lock());
  }

  private Command fullBalance() {
    return Commands.sequence(
        drive.followPath(Paths.BALANCE.get(0), true).withTimeout(4), balance(), drive.lock());
  }

  private Command justScore(GamePiece gamePiece, Side side, StartingPos startingPos) {
    return staticOdometryReset(gamePiece, side, startingPos).andThen(score(Goal.HIGH, gamePiece));
  }

  private CommandBase score(Goal goal, GamePiece gamePiece) {
    return arm.goTo(goal, () -> gamePiece).withTimeout(5).andThen(intake.outtake(gamePiece));
  }

  /** no PPL */
  private Command scoreBalance(GamePiece gamePiece) {
    return Commands.sequence(
        staticOdometryReset(GamePiece.CUBE, Side.BACK, CENTER),
        score(Goal.HIGH, gamePiece),
        arm.goTo(ArmState.stow()).withTimeout(3.5),
        fullBalance()); // TODO doesnt account for starting rotation
  }

  private Command coneLeave(StartingPos startingPos) {
    var pathGroup =
        switch (startingPos) {
          case BUMP -> Paths.CONE_LEAVE_BUMP;
          case FLAT -> Paths.CONE_LEAVE_FLAT;
          case CENTER -> Paths.CONE_LEAVE_FLAT;
        };
    return Commands.sequence(
        staticOdometryReset(GamePiece.CONE, Side.BACK, startingPos),
        score(Goal.HIGH, GamePiece.CONE),
        Commands.parallel(drive.followPath(pathGroup.get(0), false), arm.goTo(ArmState.stow())));
  }

  private Command cubeLeave(StartingPos startingPos) {
    var pathGroup =
        switch (startingPos) {
          case BUMP -> Paths.CUBE_LEAVE_BUMP;
          case FLAT -> Paths.CUBE_LEAVE_FLAT;
          case CENTER -> Paths.CONE_LEAVE_FLAT;
        };
    return Commands.sequence(
        staticOdometryReset(GamePiece.CUBE, Side.BACK, startingPos),
        score(Goal.HIGH, GamePiece.CUBE),
        Commands.parallel(drive.followPath(pathGroup.get(0), false), arm.goTo(ArmState.stow())));
  }

  private Command cubeScoreIntake() {
    return Commands.sequence(
        staticOdometryReset(GamePiece.CUBE, Side.BACK, FLAT),
        score(Goal.HIGH, GamePiece.CUBE),
        Commands.parallel(
            Commands.sequence(
                Commands.waitSeconds(0.5), drive.followPath(Paths.CUBE_INTAKE_FLAT.get(0), false)),
            frontMovingIntake(GamePiece.CUBE))); // TODO is this a cube?
  }

  private Command lowCubeLeave() {
    return Commands.sequence(
        staticOdometryReset(GamePiece.CONE, Side.FRONT, FLAT),
        arm.goTo(Goal.LOW, () -> GamePiece.CUBE),
        intake.outtake(GamePiece.CUBE),
        drive.followPath(Paths.LEAVE_FLAT_BACKWARDS.get(0), false));
  }

  /** backup: no arm */
  private Command leave(StartingPos startingPos) {
    return drive.followPath(
        (switch (startingPos) {
              case BUMP -> Paths.LEAVE_BUMP;
              case FLAT -> Paths.LEAVE_FLAT;
              case CENTER -> Paths.LEAVE_FLAT;
            })
            .get(0),
        true);
  }

  private Command staticOdometryReset(
      GamePiece gamePiece, Side scoringSide, StartingPos startingPos) {
    return staticOdometryReset(
        gamePiece, Rotation2d.fromRadians(Math.PI - scoringSide.angle), startingPos);
  }

  /** resets odometry where feild is static (doesn't depend on alliance) */
  private Command staticOdometryReset(
      GamePiece gamePiece, Rotation2d rotation, StartingPos startingPos) {
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
                      case CONE -> SCORING_POINTS_CONE
                          .get(
                              switch (startingPos) {
                                case FLAT -> 1;
                                case CENTER -> 3;
                                case BUMP -> 6;
                              })
                          .getY();
                      case CUBE -> SCORING_POINTS_CUBE
                          .get(
                              switch (startingPos) {
                                case FLAT -> 1;
                                case CENTER -> 2;
                                case BUMP -> 3;
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
    autoChooser.initSendable(builder);
  }

  public Command get() {
    return arm.setSetpoints(ArmState::initial).andThen(autoChooser.getSelected().get());
  }
}
