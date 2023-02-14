// TODO(Noah): fix code

package org.sciborgs1155.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Log;

import java.util.ArrayList;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.sciborgs1155.lib.Visualizer;
import org.sciborgs1155.robot.Ports.OI;
import org.sciborgs1155.robot.commands.Autos;
import org.sciborgs1155.robot.subsystems.Arm;
import org.sciborgs1155.robot.subsystems.Drivetrain;
import org.sciborgs1155.robot.subsystems.Elevator;
import org.sciborgs1155.robot.subsystems.Intake;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Define camera for PoseEstimation
  private final PhotonCamera cam = new PhotonCamera(Constants.CAMERA_NAME);

  @Log private final Visualizer visualizer = Visualizer.getInstance();

  // The robot's subsystems and commands are defined here...
  private final Drivetrain drive = new Drivetrain(cam);
  private final Arm arm = new Arm();
  private final Elevator elevator = new Elevator();
  private final Intake intake = new Intake();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController xbox = new CommandXboxController(OI.XBOX);
  private final CommandJoystick leftJoystick = new CommandJoystick(OI.LEFT_STICK);
  private final CommandJoystick rightJoystick = new CommandJoystick(OI.RIGHT_STICK);

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
    drive.setDefaultCommand(drive.drive(leftJoystick, rightJoystick, true));
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

    return Autos.followPath(drive, "gamer");

    // return autonSequence.stream()
    //     .reduce(Command::andThen)
    //     .orElseGet(() -> new RunCommand(() -> {}));
  }


  public Command getTankAutoCommand(){
    // make trajectory
    // for pose suplier: drive::getPose
    // ramsete controller
    RamseteCommand ramseteCommand = 
      new RamseteCommand(

      )

    public void generateTrajectry(){
      Pose2d startWaypoint = new Pose2d(0, 0, new Rotation2d(0));

      Pose2d endWaypoint = new Pose2d(6, 0, new Rotation2d(0));

      ArrayList<Translation2d> interiorWaypoints = new ArrayList<Translation2d>();
      interiorWaypoints.add(new Translation2d(2,3));
      interiorWaypoints.add(new Translation2d(1, 4));
      interiorWaypoints.add(new Translation2d( 6, 7));
      interiorWaypoints.add(new Translation2d( 23, 78));
      interiorWaypoints.add(new Translation2d( 3, 2));
    }
}
