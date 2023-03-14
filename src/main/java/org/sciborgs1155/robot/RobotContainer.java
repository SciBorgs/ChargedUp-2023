package org.sciborgs1155.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Log;
import org.sciborgs1155.lib.Vision;
import org.sciborgs1155.robot.Constants.Positions;
import org.sciborgs1155.robot.Ports.OI;
import org.sciborgs1155.robot.commands.Autos;
import org.sciborgs1155.robot.commands.Placement;
import org.sciborgs1155.robot.subsystems.Arm;
import org.sciborgs1155.robot.subsystems.Drive;
import org.sciborgs1155.robot.subsystems.Elevator;
import org.sciborgs1155.robot.subsystems.Intake;
import org.sciborgs1155.robot.util.Visualizer;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final Vision vision = new Vision();

  // The robot's subsystems and commands are defined here...
  private final Drive drive = new Drive(vision);
  // private final Elevator elevator = new Elevator(visualizer);
  // private final Arm arm = new Arm(visualizer);
  private final Intake intake = new Intake();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController operator = new CommandXboxController(OI.OPERATOR);
  private final CommandXboxController driver = new CommandXboxController(OI.DRIVER);
  private final CommandJoystick leftJoystick = new CommandJoystick(OI.LEFT_STICK);
  private final CommandJoystick rightJoystick = new CommandJoystick(OI.RIGHT_STICK);

  // command factories
  private final Placement placement = new Placement(arm, elevator);
  private final Autos autos = new Autos(drive, placement, vision, intake);

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
        drive.drive(
            () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX(), true));
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
    // rightJoystick.trigger().onTrue(intake.start(false)).onFalse(intake.stop());
    // rightJoystick.top().onTrue(intake.start(true)).onFalse(intake.stop());

    // xbox.a().onTrue(elevator.setGoal(0.3));
    // xbox.b().onTrue(elevator.setGoal(0));
    operator.a().onTrue(placement.safeToState(Positions.FRONT_INTAKE));
    operator.b().onTrue(placement.safeToState(Positions.BACK_HIGH_CONE));
    operator.x().onTrue(placement.safeToState(Positions.STOW));

    operator.leftBumper().onTrue(intake.start(false)).onFalse(intake.stop());
    operator.rightBumper().onTrue(intake.start(true)).onFalse(intake.stop());

    // xbox.povLeft().onTrue(arm.setElbowGoal(new State(0, 0)));
    // xbox.povUp().onTrue(arm.setElbowGoal(new State(1.57, 0)));
    // xbox.povRight().onTrue(arm.setElbowGoal(new State(3.14, 0)));

    // xbox.povUp().onTrue(arm.set)

    // xbox.povUp().onTrue(arm.setGoals(Rotation2d.fromDegrees(5), Rotation2d.fromDegrees(0)));
    // xbox.povDown().onTrue(arm.setGoals(Rotation2d.fromDegrees(-5), Rotation2d.fromDegrees(0)));
    // xbox.p.onTrue(arm.setVoltage(3)).onFalse(arm.setVoltage(0));
    // xbox.povDownovUp()().onTrue(arm.setVoltage(-3)).onFalse(arm.setVoltage(0));

  }

  /** A command to run when the robot is enabled */
  public Command getEnableCommand() {
    return Commands.parallel(
        elevator.setGoal(elevator.getPosition()),
        arm.setGoals(arm.getElbowPosition(), arm.getRelativeWristPosition()));
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
