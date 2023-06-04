package org.sciborgs1155.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Log;
import java.util.List;
import org.sciborgs1155.lib.CommandRobot;
import org.sciborgs1155.lib.failure.Fallible;
import org.sciborgs1155.lib.failure.FaultBuilder;
import org.sciborgs1155.lib.failure.HardwareFault;
import org.sciborgs1155.robot.Constants.Elbow;
import org.sciborgs1155.robot.Constants.Elevator;
import org.sciborgs1155.robot.Constants.Wrist;
import org.sciborgs1155.robot.Ports.OI;
import org.sciborgs1155.robot.commands.Scoring;
import org.sciborgs1155.robot.subsystems.Arm;
import org.sciborgs1155.robot.subsystems.Drive;
import org.sciborgs1155.robot.subsystems.Intake;
import org.sciborgs1155.robot.subsystems.LED;
import org.sciborgs1155.robot.subsystems.arm.ArmState;
import org.sciborgs1155.robot.subsystems.arm.ArmState.GamePiece;
import org.sciborgs1155.robot.subsystems.arm.ArmState.Level;
import org.sciborgs1155.robot.subsystems.arm.ArmState.Side;
import org.sciborgs1155.robot.vision.NoVision;
import org.sciborgs1155.robot.vision.VisionIO;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot extends CommandRobot implements Fallible, Loggable {

  private final VisionIO vision = new NoVision();

  // SUBSYSTEMS
  @Log private final Drive drive = Drive.create();

  @Log
  private final Arm arm =
      switch (Constants.ROBOT_TYPE) {
        case CHASSIS -> Arm.createNone();
        case WHIPLASH_CLAW -> Arm.createFromConfig(
            Elevator.CONFIG, Elbow.CONFIG_OLD, Wrist.CONFIG_OLD);
        case WHIPLASH_ROLLER -> Arm.createFromConfig(
            Elevator.CONFIG, Elbow.CONFIG_NEW, Wrist.CONFIG_NEW);
      };

  @Log private final Intake intake = new Intake();
  @Log private final LED led = new LED();

  // INPUT DEVICES
  private final CommandXboxController operator = new CommandXboxController(OI.OPERATOR);
  private final CommandXboxController driver = new CommandXboxController(OI.DRIVER);

  // COMMANDS
  @Log private final Scoring scoring = new Scoring(led);
  // @Log private final Autos autos = new Autos(drive, arm, intake);

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
    // addPeriodic(() -> getFaults().forEach(fault -> SmartDashboard.putData("faults/", fault)),
    // Constants.PERIOD);

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

    arm.setDefaultCommand(arm.setSetpoints(arm::getSetpoint).repeatedly().withName("holding"));

    // intake.setDefaultCommand(intake.intake(scoring.gamePiece()).withName("passive intake"));
  }

  /** Configures trigger -> command bindings */
  private void configureBindings() {
    // DRIVER INPUT
    driver.b().onTrue(drive.zeroHeading());

    driver.leftBumper().onTrue(drive.setSpeedMultiplier(0.3)).onFalse(drive.setSpeedMultiplier(1));

    driver.rightBumper().onTrue(drive.setSpeedMultiplier(0.3)).onFalse(drive.setSpeedMultiplier(1));

    // STATE SWITCHING
    operator.b().onTrue(scoring.setSide(Side.FRONT));
    operator.x().onTrue(scoring.setSide(Side.BACK));
    operator.a().onTrue(scoring.setGamePiece(GamePiece.CUBE));
    operator.y().onTrue(scoring.setGamePiece(GamePiece.CONE));

    // SCORING
    operator.povUp().onTrue(arm.goTo(() -> scoring.state(Level.HIGH)));
    operator.povRight().onTrue(arm.goTo(() -> scoring.state(Level.MID)));
    operator.povDown().onTrue(arm.goTo(() -> scoring.state(Level.LOW)));
    operator.povLeft().onTrue(arm.goTo(ArmState.STOW));

    operator.leftTrigger().onTrue(arm.goTo(() -> scoring.state(Level.SINGLE_SUBSTATION)));
    operator.rightTrigger().onTrue(arm.goTo(() -> scoring.state(Level.DOUBLE_SUBSTATION)));

    operator.rightStick().onTrue(arm.goTo(ArmState.OLD_SAFE));
    operator.leftStick().onTrue(arm.goTo(ArmState.OLD_SAFE));

    // INTAKING
    operator.leftBumper().whileTrue(intake.intake(scoring.gamePiece()));
    operator.rightBumper().whileTrue(intake.outtake(scoring.gamePiece()));

    // FAILURE MODES
    arm.onFailing(arm.kill());
    intake.onFailing(Commands.print("intake broken"));
    drive.onFailing(Commands.print("drive broken"));
  }

  @Override
  public List<HardwareFault> getFaults() {
    var builder = new FaultBuilder();
    builder.add(drive.getFaults());
    builder.add(arm.getFaults());
    builder.add(intake.getFaults());
    return builder.faults();
  }

  /** The command to run when the robot is enabled */
  public Command getEnableCommand() {
    return arm.setSetpoints(arm::getState);
  }

  /** The commamnd to be ran in autonomous */
  public Command getAutonomousCommand() {
    // return new DeferredCommand(Auto::get, drive, arm)
    //     .until(() -> !DriverStation.isAutonomous())
    //     .withName("auto");
    return Commands.none();
  }
}
