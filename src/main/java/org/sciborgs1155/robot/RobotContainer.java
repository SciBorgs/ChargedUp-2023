package org.sciborgs1155.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Log;
import org.sciborgs1155.lib.Vision;
import org.sciborgs1155.lib.Visualizer;
import org.sciborgs1155.robot.Constants.Positions;
import org.sciborgs1155.robot.Ports.OI;
import org.sciborgs1155.robot.commands.Autos;
import org.sciborgs1155.robot.commands.Placement;
import org.sciborgs1155.robot.commands.Scoring;
import org.sciborgs1155.robot.commands.Scoring.GamePiece;
import org.sciborgs1155.robot.commands.Scoring.Level;
import org.sciborgs1155.robot.commands.Scoring.Side;
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
  private final CommandXboxController operator = new CommandXboxController(OI.OPERATOR);
  private final CommandXboxController driver = new CommandXboxController(OI.DRIVER);
  // private final CommandJoystick leftJoystick = new CommandJoystick(OI.LEFT_STICK);
  // private final CommandJoystick rightJoystick = new CommandJoystick(OI.RIGHT_STICK);

  // command factories
  private final Placement placement = new Placement(arm, elevator);
  @Log private final Scoring scoring = new Scoring(drive, placement, intake, vision);
  private final Autos autos = new Autos(drive, placement, vision, intake, scoring);

  // Operator Profiles
  private enum ButtonProfile {
    PRESET,
    SETPOINT,
    VOLTAGE
  }

  private final SendableChooser<ButtonProfile> profileChooser =
      new SendableChooser<ButtonProfile>();

  private ButtonProfile getProfile() {
    return profileChooser.getSelected();
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the oblog logger
    Logger.configureLoggingAndConfig(this, false);
    // Set up profile chooser
    profileChooser.setDefaultOption("Preset Control", ButtonProfile.PRESET);
    profileChooser.addOption("Setpoint Control", ButtonProfile.SETPOINT);
    profileChooser.addOption("Voltage Control", ButtonProfile.VOLTAGE);
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
    driver.b().onTrue(drive.zeroHeading());

    // STATE SWITCHING
    operator.b().onTrue(scoring.setSide(Side.FRONT));
    operator.x().onTrue(scoring.setSide(Side.BACK));
    operator.a().onTrue(scoring.setGamePiece(GamePiece.CUBE));
    operator.y().onTrue(scoring.setGamePiece(GamePiece.CONE));

    // SCORING
    operator.povUp().onTrue(scoring.goTo(Level.HIGH));
    operator.povRight().onTrue(scoring.goTo(Level.MID));
    operator.povDown().onTrue(scoring.goTo(Level.LOW));
    operator.povLeft().onTrue(placement.safeToState(Positions.STOW));

    operator.leftTrigger().onTrue(scoring.goTo(Level.SINGLE_SUBSTATION));
    operator.rightTrigger().onTrue(scoring.goTo(Level.DOUBLE_SUBSTATION));
    // operator.povDownRight().onTrue(elevator.setGoal(0));
    // operator.povUpRight().onTrue(elevator.setGoal(.5));

    // Intaking
    operator.leftBumper().onTrue(intake.intakeTmp()).onFalse(intake.stop());
    operator.rightBumper().onTrue(intake.outtake()).onFalse(intake.stop());

    // TODO: not a todo but use this??
    // operator.leftBumper().onTrue(intake.intake());
    // operator.rightBumper().onTrue(intake.stop());

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
    // return scoring
    //     .odometryAlign(Side.FRONT, Alliance.BLUE)
    //     .andThen(scoring.score(Level.LOW, Side.FRONT));
  }
}
