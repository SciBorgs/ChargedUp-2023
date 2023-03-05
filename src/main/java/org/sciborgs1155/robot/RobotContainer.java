package org.sciborgs1155.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Log;
import org.sciborgs1155.lib.Vision;
import org.sciborgs1155.lib.Visualizer;
import org.sciborgs1155.robot.Ports.OI;
import org.sciborgs1155.robot.commands.Autos;
import org.sciborgs1155.robot.commands.Placement;
import org.sciborgs1155.robot.subsystems.Arm;
import org.sciborgs1155.robot.subsystems.Drive;
import org.sciborgs1155.robot.subsystems.Elevator;
import org.sciborgs1155.robot.subsystems.Intake;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final Vision vision = new Vision();
  @Log private final Visualizer visualizer = new Visualizer();

  // The robot's subsystems and commands are defined here...
  private final Drive drive = new Drive(vision);
  private final Elevator elevator = new Elevator(visualizer);
  private final Arm arm = new Arm(visualizer);
  private final Intake intake = new Intake();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController xbox = new CommandXboxController(OI.XBOX);
  private final CommandJoystick leftJoystick = new CommandJoystick(OI.LEFT_STICK);
  private final CommandJoystick rightJoystick = new CommandJoystick(OI.RIGHT_STICK);

  // command factories
  @Log private final Autos autos = new Autos(drive, vision);
  private final Placement placement = new Placement(arm, elevator);

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
        drive.drive(() -> -xbox.getLeftX(), leftJoystick::getX, rightJoystick::getY, true));
    // arm.setDefaultCommand(arm.setVoltage(() -> xbox.getRightY() * 3, () -> xbox.getLeftY() * 3));
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
    rightJoystick.trigger().onTrue(intake.start(false)).onFalse(intake.stop());
    // rightJoystick.top().onTrue(intake.start(true)).onFalse(intake.stop());

    xbox.a().onTrue(elevator.setGoal(0.55));
    xbox.b().onTrue(elevator.setGoal(0));
    xbox.x().onTrue(intake.start(false)).onFalse(intake.stop());
    xbox.y().onTrue(intake.start(true)).onFalse(intake.stop());

    xbox.povLeft().onTrue(arm.setElbowGoal(new State(0, 0)));
    xbox.povUp().onTrue(arm.setElbowGoal(new State(1.57, 0)));
    xbox.povRight().onTrue(arm.setElbowGoal(new State(3.14, 0)));
    
    // xbox.povUp().onTrue(arm.setGoals(Rotation2d.fromDegrees(5), Rotation2d.fromDegrees(0)));
    // xbox.povDown().onTrue(arm.setGoals(Rotation2d.fromDegrees(-5), Rotation2d.fromDegrees(0)));
    // xbox.p.onTrue(arm.setVoltage(3)).onFalse(arm.setVoltage(0));
    // xbox.povDownovUp()().onTrue(arm.setVoltage(-3)).onFalse(arm.setVoltage(0));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return drive.follow("PRAY", true, true);
    return autos.get();
    // return arm.setElbowGoal(new TrapezoidProfile.State(0.75 * Math.PI, 0));
    // return autos.get();
  }
}
