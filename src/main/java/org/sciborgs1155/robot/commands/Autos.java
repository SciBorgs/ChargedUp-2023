package org.sciborgs1155.robot.commands;

import static org.sciborgs1155.robot.Constants.Auto.*;
import static org.sciborgs1155.robot.Constants.Field.*;
import static org.sciborgs1155.robot.Constants.Positions.*;
import static org.sciborgs1155.robot.commands.Autos.StartingPos.*;
import static org.sciborgs1155.robot.util.placement.PlacementState.GamePiece.*;
import static org.sciborgs1155.robot.util.placement.PlacementState.Side.*;
import static org.sciborgs1155.robot.subsystems.Drive.loadPath;

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
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.List;
import java.util.function.Supplier;
import org.sciborgs1155.robot.subsystems.Drive;
import org.sciborgs1155.robot.subsystems.Intake;
import org.sciborgs1155.robot.util.placement.PlacementState.GamePiece;
import org.sciborgs1155.robot.util.placement.PlacementState.Side;

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
  private final Placement placement;
  private final Intake intake;

  private final SendableChooser<Supplier<Command>> autoChooser;

  public Autos(Drive drive, Placement placement, Intake intake) {
    this.drive = drive;
    this.intake = intake;
    this.placement = placement;

    autoChooser = new SendableChooser<Supplier<Command>>();
    configureAllAutos();
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
    autoChooser.addOption("cube, balance", this::cubeBalance);

    /* cube intake setup instructions:
     * gamepiece: cube
     * orientation: away from grid
     * starting location: cube scoring, flat side
     */
    autoChooser.addOption("cube, intake (flat)", this::cubeIntake);

    // backups

    /* cube score setup instructions:
     * gamepiece: cube
     * orientation: away from grid
     * starting location: cube scoring (preferably flat)
     */
    autoChooser.setDefaultOption("cube score (no drive)", () -> justScore(CUBE, FLAT));
    /* cone score setup instructions:
     * gamepiece: cone
     * orientation: away from grid
     * set starting pos: no
     * starting location: cone scoring (preferably flat)
     */
    autoChooser.addOption("cone score (no drive)", () -> justScore(CONE, FLAT));

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
    autoChooser.addOption("cone, balance", this::coneBalance);

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
    autoChooser.addOption("one meter test", () -> drive.followPath(Paths.ONE_METER_TEST.get(0), true));
    autoChooser.addOption("2 gamepeice (bump)", () -> twoGamepiece(BUMP));
  }

  private void configureAllAutos() {
    configureMainAutos();
    configureExtraAutos();
    configureTestAutos();
  }

  private Command frontMovingIntake() {
    return Commands.sequence(
        placement.goTo(FRONT_INTAKE),
        intake.intake().withTimeout(MOVING_INTAKE_TIME),
        intake.stop());
  }

  private Command initialIntake() {
    return Commands.sequence(intake.intake().withTimeout(INITIAL_INTAKE_TIME), intake.stop());
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
        staticOdometryReset(CONE, Rotation2d.fromRadians(0), startingPos),
        highConeScore(true),
        Commands.parallel(
            Commands.sequence(Commands.waitSeconds(0.5), drive.followPath(pathGroup.get(0), false)),
            frontMovingIntake()),
        Commands.parallel(drive.followPath(pathGroup.get(1), false), placement.goTo(STOW)),
        backHighCubeScore(),
        placement.goTo(SAFE));
  }

  private Command justBalance() {
    return Commands.sequence(
        staticOdometryReset(CUBE, Rotation2d.fromRadians(0), CENTER), fullBalance());
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

  private Command justScore(GamePiece gamePiece, StartingPos startingPos) {
    return Commands.sequence(
        staticOdometryReset(gamePiece, Rotation2d.fromDegrees(0), startingPos),
        switch (gamePiece) {
          case CONE -> highConeScore(true);
          case CUBE -> backHighCubeScore();
        });
  }

  private Command backHighCubeScore() {
    return Commands.sequence(placement.goTo(BACK_HIGH_CUBE).withTimeout(5), intake.outtake(CUBE));
  }

  private Command highConeScore(boolean preloaded) {
    return Commands.sequence(
        preloaded ? initialIntake() : Commands.none(),
        placement.goTo(BACK_HIGH_CONE).withTimeout(5),
        intake.outtake(CONE));
  }

  /** no PPL */
  private Command cubeBalance() {
    return Commands.sequence(
        staticOdometryReset(CUBE, BACK, CENTER),
        backHighCubeScore(),
        placement.goTo(SAFE).withTimeout(3.5),
        fullBalance());
  }

  /** no PPL */
  private Command coneBalance() {
    return Commands.sequence(
        staticOdometryReset(CONE, BACK, CENTER),
        highConeScore(true),
        placement.goTo(SAFE).withTimeout(3.5),
        fullBalance());
  }

  private Command coneLeave(StartingPos startingPos) {
    var pathGroup =
        switch (startingPos) {
          case BUMP -> Paths.CONE_LEAVE_BUMP;
          case FLAT -> Paths.CONE_LEAVE_FLAT;
          case CENTER -> Paths.CONE_LEAVE_FLAT;
        };
    return Commands.sequence(
        staticOdometryReset(CONE, BACK, startingPos),
        highConeScore(true),
        Commands.parallel(drive.followPath(pathGroup.get(0), false), placement.goTo(SAFE)));
  }

  private Command cubeLeave(StartingPos startingPos) {
    var pathGroup =
        switch (startingPos) {
          case BUMP -> Paths.CUBE_LEAVE_BUMP;
          case FLAT -> Paths.CUBE_LEAVE_FLAT;
          case CENTER -> Paths.CONE_LEAVE_FLAT;
        };
    return Commands.sequence(
        staticOdometryReset(CUBE, BACK, startingPos),
        backHighCubeScore(),
        Commands.parallel(drive.followPath(pathGroup.get(0), false), placement.goTo(SAFE)));
  }

  private Command cubeIntake() {
    return Commands.sequence(
        staticOdometryReset(CUBE, BACK, FLAT),
        backHighCubeScore(),
        Commands.parallel(
            Commands.sequence(
                Commands.waitSeconds(0.5), drive.followPath(Paths.CUBE_INTAKE_FLAT.get(0), false)),
            frontMovingIntake()),
        initialIntake());
  }

  private Command lowCubeLeave() {
    return Commands.sequence(
        staticOdometryReset(CONE, FRONT, FLAT),
        placement.goTo(FRONT_INTAKE),
        intake.outtake(CUBE),
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

  public Command staticOdometryReset(
      GamePiece gamePiece, Side scoringSide, StartingPos startingPos) {
    return staticOdometryReset(
        gamePiece, Rotation2d.fromRadians(Math.PI - scoringSide.rads()), startingPos);
  }

  /** resets odometry where feild is static (doesn't depend on alliance) */
  public Command staticOdometryReset(
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

  /** resets odometry like pp does */
  public Command PPLOdometryReset(
      GamePiece gamePiece, Rotation2d rotation, StartingPos startingPos) {
    Pose2d bluePose =
        new Pose2d(
            switch (gamePiece) {
              case CONE -> SCORING_POINTS_CONE.get(
                  switch (startingPos) {
                    case FLAT -> 1;
                    case CENTER -> 3;
                    case BUMP -> 6;
                  });
              case CUBE -> SCORING_POINTS_CUBE.get(
                  switch (startingPos) {
                    case FLAT -> 1;
                    case CENTER -> 2;
                    case BUMP -> 3;
                  });
            },
            rotation);
    return Commands.runOnce(
        () -> drive.resetOdometry(transformPoseForAllaince(bluePose, DriverStation.getAlliance())),
        drive);
  }

  private Pose2d transformPoseForAllaince(Pose2d pose, DriverStation.Alliance alliance) {
    return switch (alliance) {
      case Red -> new Pose2d(
          pose.getX(), FIELD_WIDTH_METERS - pose.getY(), pose.getRotation().times(-1));
      case Blue -> pose;
      case Invalid -> pose; // should never happen
    };
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    autoChooser.initSendable(builder);
  }

  public Command get() {
    return placement.setSetpoint(INITIAL).andThen(autoChooser.getSelected().get());
  }
}
