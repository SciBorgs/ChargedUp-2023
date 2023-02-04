package org.sciborgs1155.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.sciborgs1155.lib.Visualizer;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Constants.Dimensions;
import org.sciborgs1155.robot.Constants.Motors;
import org.sciborgs1155.robot.Constants.PlacementConstants.Elbow;
import org.sciborgs1155.robot.Constants.PlacementConstants.Wrist;
import org.sciborgs1155.robot.Ports.ClawPorts;
import org.sciborgs1155.robot.Ports.ElbowPorts;

public class Arm extends SubsystemBase {

  // Reference to a Mechanism2d for displaying the arm's movement+
  private final Visualizer visualizer;

  private final CANSparkMax wrist;
  private final RelativeEncoder wristEncoder;

  private final ProfiledPIDController wristFeedback;
  private final ArmFeedforward wristFeedforward;

  private final CANSparkMax elbowMotors;
  private final RelativeEncoder elbowEncoder;

  private final ArmFeedforward elbowFeedforward;
  private final ProfiledPIDController elbowFeedback;

  private double lastSpeed = 0.0;
  private double lastTime = Timer.getFPGATimestamp();
  private double acceleration = 0.0;

  private Rotation2d elbowGoal = new Rotation2d();
  private Rotation2d wristGoal = new Rotation2d();

  // simulation
  private final SingleJointedArmSim elbowSim;
  private final SingleJointedArmSim wristSim;

  public Arm(Visualizer visualizer) {
    this.visualizer = visualizer;

    wrist = Motors.WRIST.buildCanSparkMax(MotorType.kBrushless, ClawPorts.CLAW_WRIST);
    wristEncoder = wrist.getEncoder();

    wristFeedback =
        new ProfiledPIDController(Wrist.kP, Wrist.kI, Wrist.kD, Wrist.WRIST_CONSTRAINTS);
    wristFeedforward = new ArmFeedforward(Wrist.kS, Wrist.kG, Wrist.kV, Wrist.kA);

    elbowMotors = Motors.ELBOW.buildCanSparkMaxGearbox(MotorType.kBrushless, ElbowPorts.elbowPorts);
    elbowEncoder = elbowMotors.getEncoder();

    elbowFeedforward = new ArmFeedforward(Elbow.kS, Elbow.kG, Elbow.kV, Elbow.kA);
    elbowFeedback =
        new ProfiledPIDController(Elbow.kP, Elbow.kI, Elbow.kD, Elbow.ELBOW_CONSTRAINTS);

    elbowEncoder.setPositionConversionFactor(
        Elbow.GEAR_RATIO * Elbow.MOVEMENT_PER_SPIN); // what is movement per spin?
    elbowEncoder.setVelocityConversionFactor(Elbow.GEAR_RATIO);

    elbowSim =
        new SingleJointedArmSim(
            DCMotor.getNEO(2),
            1,
            1,
            Dimensions.FOREARM_LENGTH,
            Dimensions.ELBOW_MIN_ANGLE,
            Dimensions.ELBOW_MAX_ANGLE,
            1,
            true);

    wristSim =
        new SingleJointedArmSim(
            DCMotor.getNeo550(1),
            1,
            1,
            Dimensions.CLAW_LENGTH,
            Dimensions.WRIST_MIN_ANGLE,
            Dimensions.WRIST_MAX_ANGLE,
            1,
            true);
  }

  public void setElbowGoal(Rotation2d elbowGoal) {
    this.elbowGoal = elbowGoal;
  }

  public void setWristGoal(Rotation2d wristGoal) {
    this.wristGoal = wristGoal;
  }

  public Rotation2d getElbowgoal() {
    return elbowGoal;
  }

  public Rotation2d getWristGoal() {
    return wristGoal;
  }

  @Override
  public void periodic() {

    acceleration =
        (elbowFeedback.getSetpoint().velocity - lastSpeed) / (Timer.getFPGATimestamp() - lastTime);
    lastSpeed = elbowFeedback.getSetpoint().velocity;
    lastTime = Timer.getFPGATimestamp();

    double elbowfb = elbowFeedback.calculate(elbowEncoder.getPosition(), elbowGoal.getRadians());
    double elbowff =
        elbowFeedforward.calculate(
            elbowFeedback.getSetpoint().position,
            elbowFeedback.getSetpoint().velocity,
            acceleration);
    elbowMotors.setVoltage(elbowfb + elbowff);

    double wristfb = wristFeedback.calculate(wristEncoder.getPosition(), wristGoal.getRadians());
    double wristff =
        wristFeedforward.calculate(
            wristFeedback.getSetpoint().position,
            wristFeedback.getSetpoint().velocity,
            acceleration);
    wrist.setVoltage(wristfb + wristff);
  }

  @Override
  public void simulationPeriodic() {
    elbowSim.setInputVoltage(elbowMotors.getAppliedOutput());
    elbowSim.update(Constants.RATE);
    wristSim.setInputVoltage(wrist.getAppliedOutput());
    wristSim.update(Constants.RATE);
    visualizer.setArmAngles(
        Rotation2d.fromRadians(elbowSim.getAngleRads()),
        Rotation2d.fromRadians(wristSim.getAngleRads()));
  }
}
