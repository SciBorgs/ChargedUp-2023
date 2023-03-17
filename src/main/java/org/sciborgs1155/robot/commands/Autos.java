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
import org.sciborgs1155.lib.constants.PIDConstants;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.commands.Scoring.*;
import org.sciborgs1155.robot.subsystems.Drive;
import org.sciborgs1155.robot.subsystems.Intake;

public class Autos implements Loggable {

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

  public static com.pathplanner.lib.auto.PIDConstants PIDSciToPPL(PIDConstants pid) {
    return new com.pathplanner.lib.auto.PIDConstants(pid.p(), pid.i(), pid.d());
  }

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
            PIDSciToPPL(Constants.Drive.CARTESIAN),
            PIDSciToPPL(Constants.Drive.ANGULAR),
            drive::setModuleStates,
            eventMarkers,
            true,
            drive);

    startingPosChooser = new SendableChooser<StartingPos>();
    startingPosChooser.addOption("left", StartingPos.LEFT);
    startingPosChooser.addOption("right", StartingPos.RIGHT);
    startingPosChooser.addOption("center", StartingPos.CENTER);

    autoChooser = new SendableChooser<Supplier<Command>>();
    autoChooser.setDefaultOption("simplest drive", this::simplestDrive);
    autoChooser.addOption("simple drive", this::simpleDrive);
    autoChooser.addOption("meandering drive", this::meanderingDrive);
    autoChooser.addOption("balance", this::justBalance);
    autoChooser.addOption("goofy", this::goofy);
    autoChooser.addOption("goofyApp", this::goofyApp);
    autoChooser.addOption("score", this::highConeScore);
    autoChooser.addOption("intake", this::autoIntake);
    autoChooser.addOption("cone, cube, engage", this::coneCubeEngage);
    autoChooser.addOption("cone, cube, intake", this::coneCubeIntake);
    autoChooser.addOption("cone, balance", this::coneBalance);
  }

  private Map<String, Command> genEventMarkers() {
    return Map.ofEntries(
        Map.entry(
            "backHighCone",
            scoring
                .setGamePiece(GamePiece.CONE)
                .andThen(scoring.setSide(Side.BACK))
                .andThen(scoring.goTo(Level.HIGH))
                .andThen(placement.safeToState(STOW))),
        Map.entry(
            "frontHighCube",
            scoring
                .setGamePiece(GamePiece.CUBE)
                .andThen(scoring.setSide(Side.FRONT))
                .andThen(scoring.goTo(Level.HIGH))
                .andThen(placement.safeToState(STOW))),
        Map.entry(
            "backHighCube",
            scoring
                .setGamePiece(GamePiece.CUBE)
                .andThen(scoring.setSide(Side.BACK))
                .andThen(scoring.goTo(Level.HIGH))
                .andThen(placement.safeToState(STOW))));
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
    return followAutoPath("cone score to intake" + startingPos.suffix, true)
        .andThen(autoIntake())
        .andThen(followAutoPath("intake to cube score to balance" + startingPos.suffix, false))
        .andThen(balance(Rotation2d.fromRadians(0)));
  }

  private Command coneCubeIntake() {
    StartingPos startingPos = startingPosChooser.getSelected();
    if (startingPos == StartingPos.CENTER) {
      throw new RuntimeException("cannot do cone cube intake auto path from center");
    }
    return followAutoPath("cone score to intake" + startingPos.suffix, true)
        .andThen(autoIntake())
        .andThen(followAutoPath("intake to cube score to intake" + startingPos.suffix, false))
        .andThen(autoIntake());
  }

  private Command coneBalance() {
    if (startingPosChooser.getSelected() != StartingPos.CENTER) {
      throw new RuntimeException("cone balance path can only be done from center");
    }
    return followAutoPath("cone score to balance", true)
        .andThen(balance(Rotation2d.fromRadians(Math.PI)));
  }

  private Command justBalance() {
    if (startingPosChooser.getSelected() != StartingPos.CENTER) {
      throw new RuntimeException("just balance path can only be done from center");
    }
    return balance(Rotation2d.fromRadians(Math.PI));
  }

  private Command balance(Rotation2d rot) {
    return drive
        .drive(() -> 0.5, () -> 0, () -> rot.getRadians(), true)
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

  private Command autoIntake() {
    return scoring
        .intake(FRONT_INTAKE)
        .alongWith(drive.drive(() -> 0.1, () -> 0.1, () -> 0, false))
        .until(intake::isHoldingItem);
  }

  private Command highConeScore() {
    return scoring
        .setGamePiece(GamePiece.CONE)
        .andThen(scoring.setSide(Side.BACK))
        .andThen(scoring.goTo(Level.HIGH));
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
