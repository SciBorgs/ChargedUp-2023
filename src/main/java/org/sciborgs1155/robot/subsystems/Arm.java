package org.sciborgs1155.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.sciborgs1155.lib.Visualizer;
import org.sciborgs1155.robot.Constants.Motors;
import org.sciborgs1155.robot.Constants.PlacementConstants.Elbow;
import org.sciborgs1155.robot.Constants.PlacementConstants.Wrist;
import org.sciborgs1155.robot.Ports.ArmPorts;
import org.sciborgs1155.robot.Ports.ClawPorts;

public class Arm extends SubsystemBase {

  // Reference to a Mechanism2d for displaying the arm's movement
  private final Visualizer visualizer;

  private final CANSparkMax wrist;
  private final RelativeEncoder wristEncoder;

  // i would strongly consider an additional ArmFeedforward for the wrist, it's quite heavy
  // in addition, to make that work, we'd want a ProfiledPIDController instead of a normal
  // PIDController to handle integration of a TrapezoidProfile
  private final PIDController wristFeedback;

  private final CANSparkMax armMotors; // is the arm only controlled by one motor?
  private final RelativeEncoder armEncoder; // which arm?

  private final ArmFeedforward armFeedforward; // ^
  private final ProfiledPIDController armFeedback; // ^

  /*
   * What mechanism does this goal apply to???
   */
  private Rotation2d goal = new Rotation2d();

  /*
   * We want another goal for the second joint
   *
   * generally we'll name things as elbow and wrist
   */

  public Arm(Visualizer visualizer) {

    this.visualizer = visualizer;

    // great!
    wrist = Motors.WRIST.buildCanSparkMax(MotorType.kBrushless, ClawPorts.CLAW_WRIST);
    wristEncoder = wrist.getEncoder();

    wristFeedback = new PIDController(Wrist.kP, Wrist.kI, Wrist.kD);

    armMotors = Motors.ELBOW.buildCanSparkMaxGearbox(MotorType.kBrushless, ArmPorts.armPorts);
    armEncoder = armMotors.getEncoder();

    armFeedforward = new ArmFeedforward(Elbow.kS, Elbow.kG, Elbow.kV, Elbow.kA);
    armFeedback = new ProfiledPIDController(Elbow.kP, Elbow.kI, Elbow.kD, Elbow.CONSTRAINTS);

    armEncoder.setPositionConversionFactor(
        Elbow.GEAR_RATIO * Elbow.MOVEMENTPERSPIN); // what is movement per spin?
    armEncoder.setVelocityConversionFactor(Elbow.GEAR_RATIO);
  }

  /*
   * Set the goal for which mechanism?
   */
  public void setGoal(Rotation2d goal) {
    this.goal = goal;
  }

  @Override
  public void periodic() {
    double fb = armFeedback.calculate(armEncoder.getPosition(), goal.getRadians());
    double ff =
        armFeedforward.calculate(
            armFeedback.getSetpoint().position, armFeedback.getSetpoint().velocity);
    armMotors.setVoltage(fb + ff);

    /*
     * possibly add acceleration calculations for the arm feedforward
     * I could add a util class to help keep this neat
     */

    // pid.calculate(measurement, setpoint), where measurement is your current position and setpoint
    // is your desired position
    // the measurement is good, but we don't ever set setpoint, meaning it'll consistently attempt
    // to reach 0
    wrist.set(wristFeedback.calculate(wristEncoder.getPosition()));
  }

  /**
   * This is not C#, methods should be in camelCase, not PascalCase - we should only set the motor's
   * value in {@link this#periodic()} - generally, we also organize subsystems by putting setter
   * methods (methods that set variables, such as a setpoint here) above periodic
   */
  public void DisableWrist() {
    wrist.set(0);
  }
}
