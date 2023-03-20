package org.sciborgs1155.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Log;
import java.util.function.Supplier;
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
import org.sciborgs1155.robot.subsystems.LED;
import org.sciborgs1155.robot.util.Vision;
import org.sciborgs1155.robot.util.Visualizer;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer implements Loggable {

  // Vision instance
  private final Vision vision = new Vision();

  // Placement visualizer
  @Log private final Visualizer visualizer = new Visualizer();

  // Subsystems
  @Log private final Drive drive = new Drive(vision);
  @Log private final Elevator elevator = new Elevator(visualizer);
  @Log private final Arm arm = new Arm(visualizer);
  @Log private final Intake intake = new Intake();
  @Log private final LED led = new LED();

  // Input devices
  private final CommandXboxController operator = new CommandXboxController(OI.OPERATOR);
  private final CommandXboxController driver = new CommandXboxController(OI.DRIVER);

  // Command factories
  private final Placement placement = new Placement(arm, elevator);
  @Log private final Scoring scoring = new Scoring(drive, placement, led);

  @Log(name = "starting position chooser")
  private final Autos autos = new Autos(drive, placement, intake);

  // Auto choosers
  @Log(name = "auto path chooser")
  private final SendableChooser<Supplier<Command>> autoChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the oblog logger
    Logger.configureLoggingAndConfig(this, false);
    // Configure auto chooser options
    configureAutoChoosers();
    // Configure the trigger bindings
    configureBindings();
    // Configure subsystem default commands
    configureSubsystemDefaults();
  }

  private void configureAutoChoosers() {
    autoChooser.addOption("balance", autos::justBalance);
    autoChooser.addOption("cone score", autos::highConeScore);
    autoChooser.addOption("back cube score", autos::backHighCubeScore);
    autoChooser.addOption("front cube score", autos::frontHighCubeScore);
    autoChooser.addOption("cone, cube, engage", autos::coneCubeEngage);
    autoChooser.addOption("cone, cube, intake", autos::coneCubeIntake);
    autoChooser.addOption("cube, balance", autos::cubeBalance);
    autoChooser.addOption("cone leave", autos::coneLeave);
    autoChooser.addOption("cube leave", autos::cubeLeave);
    autoChooser.addOption("cone/cube leave (no ppl)", autos::scoreLeaveNoPPL);
  }

  private void configureSubsystemDefaults() {
    drive.setDefaultCommand(
        drive
            .drive(
                () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX(), true)
            .withName("teleop driving"));
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

    // INTAKING
    operator.leftBumper().onTrue(intake.intake()).onFalse(intake.stop());
    operator.rightBumper().onTrue(intake.outtake()).onFalse(intake.stop());
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
    return autoChooser.getSelected().get();
  }
}
