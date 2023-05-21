package org.sciborgs1155.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Log;
import org.sciborgs1155.lib.CommandRobot;
import org.sciborgs1155.lib.DeferredCommand;
import org.sciborgs1155.robot.Constants.Drive.SpeedMultiplier;
import org.sciborgs1155.robot.Constants.Positions;
import org.sciborgs1155.robot.Ports.OI;
import org.sciborgs1155.robot.commands.Autos;
import org.sciborgs1155.robot.commands.Scoring;
import org.sciborgs1155.robot.subsystems.Arm;
import org.sciborgs1155.robot.subsystems.Drive;
import org.sciborgs1155.robot.subsystems.Intake;
import org.sciborgs1155.robot.subsystems.LED;
import org.sciborgs1155.robot.util.RealVision;
import org.sciborgs1155.robot.util.SimVision;
import org.sciborgs1155.robot.util.VisionIO;
import org.sciborgs1155.robot.util.placement.PlacementState.GamePiece;
import org.sciborgs1155.robot.util.placement.PlacementState.Level;
import org.sciborgs1155.robot.util.placement.PlacementState.Side;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot extends CommandRobot implements Loggable {

  private final VisionIO vision = isReal() ? new RealVision() : new SimVision();

  // SUBSYSTEMS
  @Log private final Drive drive = new Drive();
  @Log private final Arm arm = new Arm();
  @Log private final Intake intake = new Intake();
  @Log private final LED led = new LED();

  // INPUT DEVICES
  private final CommandXboxController operator = new CommandXboxController(OI.OPERATOR);
  private final CommandXboxController driver = new CommandXboxController(OI.DRIVER);

  // COMMANDS
  // private final Placement placement = new Placement(arm, elevator);
  @Log private final Scoring scoring = new Scoring(led);
  @Log private final Autos autos = new Autos(drive, arm, intake);

  /** The robot contains subsystems, OI devices, and commands. */
  public Robot() {
    super(Constants.PERIOD);

    configureGameBehavior();
    configureBindings();
    configureSubsystemDefaults();
  }

  /** Configures basic behavior during different parts of the game. */
  private void configureGameBehavior() {
    if (isSimulation()) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }

    Logger.configureLoggingAndConfig(this, false);

    DataLogManager.start();

    addPeriodic(Logger::updateEntries, Constants.PERIOD);

    addPeriodic(() -> drive.updateEstimates(vision.getPoseEstimates(drive.getPose())), 0.5);

    autonomous().onTrue(getAutonomousCommand());

    teleop().onTrue(getEnableCommand());
  }

  /**
   * Configures subsystem default commands. Default commands are scheduled when no other command is
   * running on a subsystem.
   */
  private void configureSubsystemDefaults() {
    drive.setDefaultCommand(
        drive
            .drive(() -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX())
            .withName("teleop driving"));

    arm.setDefaultCommand(arm.setSetpoints(arm::getSetpoint).repeatedly());

    intake.setDefaultCommand(intake.set(Constants.Intake.DEFAULT_SPEED).withName("passive intake"));
  }

  /** Configures trigger -> command bindings */
  private void configureBindings() {
    // DRIVER INPUT
    driver.b().onTrue(drive.zeroHeading());

    driver
        .leftBumper()
        .onTrue(drive.setSpeedMultiplier(SpeedMultiplier.SLOW))
        .onFalse(drive.setSpeedMultiplier(SpeedMultiplier.NORMAL));

    driver
        .rightBumper()
        .onTrue(drive.setSpeedMultiplier(SpeedMultiplier.SLOW))
        .onFalse(drive.setSpeedMultiplier(SpeedMultiplier.NORMAL));

    // STATE SWITCHING
    operator.b().onTrue(scoring.setSide(Side.FRONT));
    operator.x().onTrue(scoring.setSide(Side.BACK));
    operator.a().onTrue(scoring.setGamePiece(GamePiece.CUBE));
    operator.y().onTrue(scoring.setGamePiece(GamePiece.CONE));

    // SCORING
    operator.povUp().onTrue(arm.goTo(() -> scoring.state(Level.HIGH)));
    operator.povRight().onTrue(arm.goTo(() -> scoring.state(Level.MID)));
    operator.povDown().onTrue(arm.goTo(() -> scoring.state(Level.LOW)));
    operator.povLeft().onTrue(arm.goTo(Positions.STOW));

    operator.leftTrigger().onTrue(arm.goTo(() -> scoring.state(Level.SINGLE_SUBSTATION)));
    operator.rightTrigger().onTrue(arm.goTo(() -> scoring.state(Level.DOUBLE_SUBSTATION)));

    operator.rightStick().onTrue(arm.goTo(Positions.SAFE));
    operator.leftStick().onTrue(arm.goTo(Positions.SAFE));

    // INTAKING
    operator.leftBumper().onTrue(intake.intake(scoring.gamePiece())).onFalse(intake.stop());
    operator.rightBumper().onTrue(intake.outtake(scoring.gamePiece())).onFalse(intake.stop());

    // FAILURE MODES
    // arm.onElbowFailing().onTrue(placement.setStopped(true));
    // elevator.onFailing().onTrue(placement.setStopped(true));
  }

  /** The command to run when the robot is enabled */
  public Command getEnableCommand() {
    return arm.setSetpoints(arm::getState);
  }

  /** The commamnd to be ran in autonomous */
  public Command getAutonomousCommand() {
    return new DeferredCommand(autos::get, drive, arm)
        .until(() -> !DriverStation.isAutonomous())
        .withName("auto");
  }
}
