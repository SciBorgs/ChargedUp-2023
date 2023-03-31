package org.sciborgs1155.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Log;
import java.util.function.Supplier;
import org.sciborgs1155.robot.Constants.Drive.SpeedMultiplier;
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
    // autoChooser.addOption("balance", autos::justBalance);
    autoChooser.addOption("balance (no ppl)", autos::balanceNoPPL);
    autoChooser.addOption("high cone", autos::highConeScore);
    autoChooser.addOption("back high cube", autos::backHighCubeScore);
    autoChooser.addOption("front high cube", autos::frontHighCubeScore);
    autoChooser.addOption("back high cube -> intake", autos::cubeIntake);
    autoChooser.addOption("score cone, score cube, engage", autos::coneCubeEngage);
    autoChooser.addOption("score cone, score cube, intake", autos::coneCubeIntake);
    autoChooser.addOption("back high cube -> engage", autos::cubeBalance);
    autoChooser.setDefaultOption("high cone -> leave comm", autos::coneLeave);
    autoChooser.addOption("back high cube -> leave comm", autos::cubeLeave);
    // autoChooser.addOption("no ppl: back high cone/cube -> leave comm", autos::scoreLeaveNoPPL);
    autoChooser.addOption("none", Commands::none);
    autoChooser.addOption("cube balance NO PPL", autos::scoreBalanceNoPPL);
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

    // SPEED SWITCHING
    driver
        .leftBumper()
        .onTrue(drive.setSpeedMultiplier(SpeedMultiplier.SLOW))
        .onFalse(drive.setSpeedMultiplier(SpeedMultiplier.NORMAL));
    driver
        .rightBumper()
        .onTrue(drive.setSpeedMultiplier(SpeedMultiplier.MAX))
        .onFalse(drive.setSpeedMultiplier(SpeedMultiplier.NORMAL));

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
  public Supplier<Command> getEnableCommand() {
    return () -> Commands.none();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return placement.followTrajectory(new Parameters(new PlacementState(0, null, null)));
    return autoChooser.getSelected().get();
  }
}
