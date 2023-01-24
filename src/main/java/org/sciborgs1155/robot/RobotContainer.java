package org.sciborgs1155.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import io.github.oblarg.oblog.Logger;
import java.util.List;
import org.sciborgs1155.robot.Ports.OI;
import org.sciborgs1155.robot.commands.Autos;
import org.sciborgs1155.robot.subsystems.Drivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain drive = new Drivetrain();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController xbox = new CommandXboxController(OI.XBOX);
  private final Joystick leftJoystick = new Joystick(OI.LEFT_STICK);
  private final Joystick rightJoystick = new Joystick(OI.RIGHT_STICK);

  private List<Command> autonSequence =
      List.of(Autos.mobility(drive), Autos.followPath(drive, "New Path"));

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the oblog logger
    Logger.configureLoggingAndConfig(this, false);
    // Configure the trigger bindings
    configureBindings();
    // Configure subsystem default commands
    configureSubsystemDefaults();
  }

  private void configureSubsystemDefaults() {
    drive.setDefaultCommand(
        new RunCommand(
            () -> {
              drive.drive(
                  MathUtil.applyDeadband(-leftJoystick.getY(), Constants.DEADBAND),
                  MathUtil.applyDeadband(-leftJoystick.getX(), Constants.DEADBAND),
                  MathUtil.applyDeadband(-rightJoystick.getX(), Constants.DEADBAND),
                  false);
            },
            drive));
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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    // .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    // xbox.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Chain all commands given by autoSequence
    return autonSequence.stream()
        .reduce(Command::andThen)
        .orElseGet(() -> new RunCommand(() -> {}));
  }
}
