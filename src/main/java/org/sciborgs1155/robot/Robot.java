package org.sciborgs1155.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Log;
import org.sciborgs1155.lib.CommandRobot;
import org.sciborgs1155.lib.DeferredCommand;
import org.sciborgs1155.robot.Constants.Drive.SpeedMultiplier;
import org.sciborgs1155.robot.Constants.Positions;
import org.sciborgs1155.robot.Ports.OI;
import org.sciborgs1155.robot.commands.Autos;
import org.sciborgs1155.robot.commands.Placement;
import org.sciborgs1155.robot.commands.Scoring;
import org.sciborgs1155.robot.subsystems.Arm;
import org.sciborgs1155.robot.subsystems.Drive;
import org.sciborgs1155.robot.subsystems.Elevator;
import org.sciborgs1155.robot.subsystems.Intake;
import org.sciborgs1155.robot.subsystems.LED;
import org.sciborgs1155.robot.util.Vision;
import org.sciborgs1155.robot.util.Vision.Mode;
import org.sciborgs1155.robot.util.Visualizer;
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

  // Vision instance
  private final Vision vision = new Vision(Mode.SIM);
  @Log private final Visualizer position = new Visualizer(new Color8Bit(255, 0, 0));
  @Log private final Visualizer setpoint = new Visualizer(new Color8Bit(0, 0, 255));

  // Subsystems
  @Log private final Drive drive = new Drive(vision);
  @Log private final Elevator elevator = new Elevator(position, setpoint);
  @Log private final Arm arm = new Arm(position, setpoint);
  @Log private final Intake intake = new Intake();
  @Log private final LED led = new LED();

  // Input devices
  private final CommandXboxController operator = new CommandXboxController(OI.OPERATOR);
  private final CommandXboxController driver = new CommandXboxController(OI.DRIVER);

  // Command factories
  private final Placement placement = new Placement(arm, elevator);
  @Log private final Scoring scoring = new Scoring(placement, led);

  @Log(name = "auto path chooser!")
  private final Autos autos = new Autos(drive, placement, intake);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public Robot() {
    super(Constants.RATE);

    if (isSimulation()) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }

    Logger.configureLoggingAndConfig(this, false);

    configureBindings();
    configureSubsystemDefaults();

    robot()
        .onTrue(Commands.runOnce(DataLogManager::start))
        .whileTrue(Commands.runOnce(Logger::updateEntries));

    autonomous().onTrue(getAutonomousCommand().until(() -> !DriverStation.isAutonomous()));

    teleop().onTrue(getEnableCommand());
  }

  private void configureSubsystemDefaults() {
    drive.setDefaultCommand(
        drive
            .drive(
                () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX(), true)
            .withName("teleop driving"));

    intake.setDefaultCommand(intake.set(Constants.Intake.DEFAULT_SPEED).withName("passive intake"));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    driver.b().onTrue(drive.zeroHeading());

    // SPEED SWITCHING
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
    operator.povUp().onTrue(placement.goTo(() -> scoring.state(Level.HIGH)));
    operator.povRight().onTrue(placement.goTo(() -> scoring.state(Level.MID)));
    operator.povDown().onTrue(placement.goTo(() -> scoring.state(Level.LOW)));
    operator.povLeft().onTrue(placement.goTo(Positions.STOW));

    operator.leftTrigger().onTrue(placement.goTo(() -> scoring.state(Level.SINGLE_SUBSTATION)));
    operator.rightTrigger().onTrue(placement.goTo(() -> scoring.state(Level.DOUBLE_SUBSTATION)));

    operator.rightStick().onTrue(placement.goTo(Positions.BALANCE));
    operator.leftStick().onTrue(placement.goTo(Positions.BALANCE));

    // INTAKING
    operator.leftBumper().onTrue(intake.intake()).onFalse(intake.stop());
    operator.rightBumper().onTrue(intake.outtake()).onFalse(intake.stop());

    arm.onElbowFailing().onTrue(placement.setStopped(true));
    elevator.onFailing().onTrue(placement.setStopped(true));
  }

  /** The command to run when the robot is enabled */
  public Command getEnableCommand() {
    return new DeferredCommand(() -> placement.setSetpoint(placement.state()), arm, elevator);
  }

  /** The commamnd to be ran in autonomous */
  public Command getAutonomousCommand() {
    return new DeferredCommand(() -> autos.get());
  }
}
