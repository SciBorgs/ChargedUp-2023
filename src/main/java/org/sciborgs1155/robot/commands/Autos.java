package org.sciborgs1155.robot.commands;

import static org.sciborgs1155.robot.Constants.Auto.*;
import static org.sciborgs1155.robot.Constants.Drive.*;
import static org.sciborgs1155.robot.Constants.Field.*;
import static org.sciborgs1155.robot.Constants.Positions.*;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
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
import java.util.Map;
import java.util.function.Supplier;
import org.sciborgs1155.robot.subsystems.Arm;
import org.sciborgs1155.robot.subsystems.Drive;
import org.sciborgs1155.robot.subsystems.Intake;
import org.sciborgs1155.robot.subsystems.arm.ArmState.GamePiece;

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
  }

  private final Drive drive;
  private final Arm arm;
  private final Intake intake;

  private final SendableChooser<Supplier<Command>> autoChooser;

  private final SwerveAutoBuilder builder;

  private final Map<String, Command> eventMarkers;

  public Autos(Drive drive, Arm arm, Intake intake) {
    this.drive = drive;
    this.intake = intake;
    this.arm = arm;

    eventMarkers =
        Map.ofEntries(
            Map.entry("backHighCone", arm.goTo(BACK_HIGH_CONE)),
            Map.entry("backHighCube", arm.goTo(BACK_HIGH_CUBE)),
            Map.entry("frontHighCube", arm.goTo(FRONT_HIGH_CUBE)),
            Map.entry("outtakeCone", outtake(GamePiece.CONE)),
            Map.entry("outtakeCube", outtake(GamePiece.CUBE)),
            Map.entry("frontIntake", frontMovingIntake()),
            Map.entry("stow", arm.goTo(STOW)),
            Map.entry("balanceState", arm.goTo(SAFE)),
            Map.entry("initialIntake", initialIntake()));

    autoChooser = new SendableChooser<Supplier<Command>>();
    configureMainAutos();

    builder =
        new SwerveAutoBuilder(
            drive::getPose,
            drive::resetOdometry,
            drive.kinematics,
            TRANSLATION.toPPL(),
            ROTATION.toPPL(),
            drive::setModuleStates,
            eventMarkers,
            true,
            drive);
  }

  private void configureMainAutos() {
    /* 2 gamepiece setup instructions:
     * gamepiece: cone
     * orientation: away from grid
     * starting position: cone scoring, flat side
     */
    autoChooser.addOption("2 gamepiece (flat)", () -> twoGamepiece(StartingPos.FLAT));

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
    autoChooser.setDefaultOption(
        "cube score (no drive)", () -> backHighCubeScore(StartingPos.FLAT));

    /* cone score setup instructions:
     * gamepiece: cone
     * orientation: away from grid
     * set starting pos: no
     * starting location: cone scoring (preferably flat)
     */
    autoChooser.addOption("cone score (no drive)", () -> highConeScore(StartingPos.FLAT));

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
    autoChooser.addOption("cone, leave (flat)", () -> coneLeave(StartingPos.FLAT));

    /* cone leave setup instructions:
     * gamepiece: cone
     * orientation: away from grid
     * starting location: cone scoring, bump side
     */
    autoChooser.addOption("cone, leave (bump)", () -> coneLeave(StartingPos.BUMP));

    /* cube leave setup instructions:
     * gamepiece: cube
     * orientation: away from grid
     * starting location: cube scoring, flat side
     */
    autoChooser.addOption("cube, leave (flat)", () -> cubeLeave(StartingPos.FLAT));

    /* cube leave setup instructions:
     * gamepiece: cube
     * orientation: away from grid
     * starting location: cube scoring, bump side
     */
    autoChooser.addOption("cube, leave (bump)", () -> cubeLeave(StartingPos.BUMP));

    // backups

    /* leave setup instructions:
     * gamepiece: none
     * orientation: away from grid
     * set starting pos: yes
     * starting location: against grid, to one side (it should have a clear path straight forward)
     */
    autoChooser.addOption("flat leave (no arm)", () -> leave(StartingPos.FLAT));

    /* leave setup instructions:
     * gamepiece: none
     * orientation: away from grid
     * set starting pos: yes
     * starting location: against grid, to one side (it should have a clear path straight forward)
     */
    autoChooser.addOption("bump leave (no arm)", () -> leave(StartingPos.BUMP));

    autoChooser.addOption("low cube, leave (flat)", this::lowCubeLeave);
  }

  private void configureTestAutos() {
    autoChooser.addOption("one meter test", () -> builder.fullAuto(Paths.ONE_METER_TEST));
    autoChooser.addOption("2 gamepeice (bump)", () -> twoGamepiece(StartingPos.BUMP));
  }

  private void configureAllAutos() {
    configureMainAutos();
    configureExtraAutos();
    configureTestAutos();
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

  private Command outtake(GamePiece gamePiece) {
    return intake
        .outtake(gamePiece)
        .withTimeout(
            switch (gamePiece) {
              case CONE -> CONE_OUTTAKE_TIME;
              case CUBE -> CUBE_OUTTAKE_TIME;
            });
  }

  private Command frontMovingIntake() {
    return Commands.sequence(
        arm.goTo(FRONT_INTAKE), intake.intake(GamePiece.CUBE).withTimeout(MOVING_INTAKE_TIME));
  }

  private Command initialIntake() {
    return Commands.sequence(intake.intake(GamePiece.CUBE).withTimeout(INITIAL_INTAKE_TIME));
  }

  private static List<PathPlannerTrajectory> loadPath(String pathName) {
    return PathPlanner.loadPathGroup(pathName, CONSTRAINTS);
  }

  /** back cone, cube intake, back cube */
  public Command twoGamepiece(StartingPos startingPos) {
    return builder.fullAuto(
        switch (startingPos) {
          case BUMP -> Paths.TWO_GAMEPIECE_BUMP;
          case FLAT -> Paths.TWO_GAMEPIECE_FLAT;
          case CENTER -> Paths.TWO_GAMEPIECE_FLAT;
        });
  }

  private Command justBalance() {
    return Commands.sequence(
        defaultOdometryReset(GamePiece.CUBE, Rotation2d.fromRadians(0), StartingPos.CENTER),
        fullBalance());
  }

  private Command fullBalance() {
    return drive
        .follow("balance", true, true)
        .withTimeout(4)
        .andThen(balance())
        .andThen(drive.lock())
        .withName("balance auto");
  }

  private Command highConeScore() {
    return highConeScore(StartingPos.FLAT);
  }

  private Command highConeScore(StartingPos startingPos) {
    return Commands.sequence(
        defaultOdometryReset(GamePiece.CONE, Rotation2d.fromRadians(0), startingPos),
        initialIntake(),
        arm.goTo(BACK_HIGH_CONE).withTimeout(5),
        outtake(GamePiece.CONE));
  }

  private Command backHighCubeScore() {
    return backHighCubeScore(StartingPos.FLAT);
  }

  private Command backHighCubeScore(StartingPos startingPos) {
    return Commands.sequence(
        defaultOdometryReset(GamePiece.CUBE, Rotation2d.fromRadians(0), startingPos),
        arm.goTo(BACK_HIGH_CUBE).withTimeout(5),
        outtake(GamePiece.CUBE));
  }

  /** no PPL */
  private Command cubeBalance() {
    return Commands.sequence(
        backHighCubeScore(StartingPos.CENTER), arm.goTo(SAFE).withTimeout(3.5), fullBalance());
  }

  /** no PPL */
  private Command coneBalance() {
    return Commands.sequence(
        highConeScore(StartingPos.CENTER), arm.goTo(SAFE).withTimeout(3.5), fullBalance());
  }

  private Command coneLeave(StartingPos startingPos) {
    return builder.fullAuto(
        switch (startingPos) {
          case BUMP -> Paths.CONE_LEAVE_BUMP;
          case FLAT -> Paths.CONE_LEAVE_FLAT;
          case CENTER -> Paths.CONE_LEAVE_FLAT;
        });
  }

  private Command cubeLeave(StartingPos startingPos) {
    return builder.fullAuto(
        switch (startingPos) {
          case BUMP -> Paths.CUBE_LEAVE_BUMP;
          case FLAT -> Paths.CUBE_LEAVE_FLAT;
          case CENTER -> Paths.CONE_LEAVE_FLAT;
        });
  }

  private Command cubeIntake() {
    return builder.fullAuto(Paths.CUBE_INTAKE_FLAT);
  }

  private Command lowCubeLeave() {
    return Commands.sequence(
        arm.goTo(FRONT_INTAKE),
        outtake(GamePiece.CUBE),
        builder.fullAuto(Paths.LEAVE_FLAT_BACKWARDS));
  }

  /** backup: no arm */
  private Command leave(StartingPos startingPos) {
    return builder.fullAuto(
        switch (startingPos) {
          case BUMP -> Paths.LEAVE_BUMP;
          case FLAT -> Paths.LEAVE_FLAT;
          case CENTER -> Paths.LEAVE_FLAT;
        });
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
  public Command defaultOdometryReset(
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
    return arm.setSetpoints(() -> INITIAL).andThen(autoChooser.getSelected().get());
  }
}
