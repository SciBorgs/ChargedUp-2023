package org.sciborgs1155.robot.subsystems;

import static org.sciborgs1155.robot.Constants.Arm.*;
import static org.sciborgs1155.robot.Ports.Arm.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import org.sciborgs1155.lib.Derivative;
import org.sciborgs1155.lib.Visualizer;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Constants.Dimensions;
import org.sciborgs1155.robot.Constants.Motors;

public class Arm extends SubsystemBase implements Loggable, AutoCloseable {

  public enum Side {
    FRONT,
    BACK;
  }

  @Log(name = "wrist applied output", methodName = "getAppliedOutput")
  private final CANSparkMax wrist = Motors.WRIST.build(MotorType.kBrushless, WRIST_MOTOR);

  @Log(name = "elbow applied output", methodName = "getAppliedOutput")
  private final CANSparkMax elbowLead = Motors.ELBOW.build(MotorType.kBrushless, LEFT_ELBOW_MOTOR);

  private final CANSparkMax elbowFollow =
      Motors.ELBOW.build(MotorType.kBrushless, RIGHT_ELBOW_MOTOR);

  @Log(name = "wrist velocity", methodName = "getVelocity")
  private final RelativeEncoder wristEncoder = wrist.getEncoder();

  @Log(name = "elbow velocity", methodName = "getVelocity")
  private final RelativeEncoder elbowEncoder = elbowLead.getEncoder();

  @Log
  private final ProfiledPIDController wristFeedback =
      new ProfiledPIDController(Wrist.kP, Wrist.kI, Wrist.kD, Wrist.CONSTRAINTS);

  @Log
  private final ProfiledPIDController elbowFeedback =
      new ProfiledPIDController(Elbow.kP, Elbow.kI, Elbow.kD, Elbow.CONSTRAINTS);

  private final ArmFeedforward wristFeedforward =
      new ArmFeedforward(Wrist.kS, Wrist.kG, Wrist.kV, Wrist.kA);
  private final ArmFeedforward elbowFeedforward =
      new ArmFeedforward(Elbow.kS, Elbow.kG, Elbow.kV, Elbow.kA);

  @Log(name = "wrist acceleration", methodName = "getLastOutput")
  private final Derivative wristAccel = new Derivative();

  @Log(name = "elbow acceleration", methodName = "getLastOutput")
  private final Derivative elbowAccel = new Derivative();

  private final SingleJointedArmSim elbowSim =
      new SingleJointedArmSim(
          DCMotor.getNEO(2),
          5,
          SingleJointedArmSim.estimateMOI(Dimensions.FOREARM_LENGTH, 2),
          Dimensions.FOREARM_LENGTH,
          Dimensions.ELBOW_MIN_ANGLE,
          Dimensions.ELBOW_MAX_ANGLE,
          true);
  private final SingleJointedArmSim wristSim =
      new SingleJointedArmSim(
          DCMotor.getNeo550(1),
          5,
          SingleJointedArmSim.estimateMOI(Dimensions.CLAW_LENGTH, 1),
          Dimensions.CLAW_LENGTH,
          Dimensions.WRIST_MIN_ANGLE,
          Dimensions.WRIST_MAX_ANGLE,
          true);

  public Arm() {
    elbowFollow.follow(elbowLead);

    elbowEncoder.setPositionConversionFactor(Elbow.GEAR_RATIO * Elbow.MOVEMENT_PER_SPIN);
    elbowEncoder.setVelocityConversionFactor(Elbow.GEAR_RATIO);
  }

  /** Elbow is at goal */
  @Log(name = "elbow at goal")
  public boolean atElbowGoal() {
    return elbowFeedback.atGoal();
  }

  /** Wrist is at goal */
  @Log(name = "wrist at goal")
  public boolean atWrsitGoal() {
    return wristFeedback.atGoal();
  }

  /** The side of the robot that the arm is currently at */
  public Side getSide() {
    return getElbowPosition().getCos() > 0 ? Side.FRONT : Side.BACK;
  }

  /** Elbow position relative to the chassis */
  @Log(name = "elbow position", methodName = "getDegrees")
  public Rotation2d getElbowPosition() {
    return Rotation2d.fromRadians(elbowEncoder.getPosition());
  }

  /** Wrist position relative to the forearm */
  @Log(name = "wrist relative positon", methodName = "getDegrees")
  public Rotation2d getRelativeWristPosition() {
    return Rotation2d.fromRadians(wristEncoder.getPosition());
  }

  /** Wrist position relative to chassis */
  @Log(name = "wrist absolute positon", methodName = "getDegrees")
  public Rotation2d getAbsoluteWristPosition() {
    return getRelativeWristPosition().plus(getElbowPosition());
  }

  /** Elbow goal relative to the chassis */
  @Log(name = "elbow goal", methodName = "getDegrees")
  public Rotation2d getElbowGoal() {
    return Rotation2d.fromRadians(elbowFeedback.getGoal().position);
  }

  /** Wrist goal relative to forearm */
  @Log(name = "wrist relative goal", methodName = "getDegrees")
  public Rotation2d getRelativeWristGoal() {
    return Rotation2d.fromRadians(wristFeedback.getGoal().position);
  }

  /** Wrist goal relative to the chassis */
  @Log(name = "wrist absolute goal", methodName = "getDegrees")
  public Rotation2d getAbsoluteWristGoal() {
    return getRelativeWristGoal().plus(getElbowGoal());
  }

  /** Sets elbow goal relative to the chassis */
  public Command setElbowGoal(Rotation2d goal) {
    return runOnce(() -> elbowFeedback.setGoal(goal.getRadians()));
  }

  /** Sets wrist goal relative to the chassis, uses {@link #getElbowPosition()} as reference */
  public Command setAbsoluteWristGoal(Rotation2d goal) {
    return runOnce(() -> wristFeedback.setGoal(goal.plus(getElbowPosition()).getRadians()));
  }

  /** Sets wrist goal relative to the forearm */
  public Command setRelativeWristGoal(Rotation2d goal) {
    return runOnce(() -> wristFeedback.setGoal(goal.getRadians()));
  }

  @Override
  public void periodic() {
    double elbowfb = elbowFeedback.calculate(elbowEncoder.getPosition());
    double elbowff =
        elbowFeedforward.calculate(
            elbowFeedback.getSetpoint().position,
            elbowFeedback.getSetpoint().velocity,
            elbowAccel.calculate(elbowFeedback.getSetpoint().velocity));
    elbowLead.setVoltage(elbowfb + elbowff);

    // wrist feedback is calculated using an absolute angle setpoint, rather than a relative one
    // this means the extra voltage calculated to cancel out gravity is kG * cos(θ + ϕ), where θ is
    // the elbow setpoint and ϕ is the wrist setpoint
    // the elbow angle is used as a setpoint instead of current position because we're using a
    // profiled pid controller, which means setpoints are achievable states, rather than goals
    double wristfb = wristFeedback.calculate(wristEncoder.getPosition());
    double wristff =
        wristFeedforward.calculate(
            wristFeedback.getSetpoint().position + elbowFeedback.getSetpoint().position,
            wristFeedback.getSetpoint().velocity,
            wristAccel.calculate(wristFeedback.getSetpoint().velocity));
    wrist.setVoltage(wristfb + wristff);

    Visualizer.getInstance().setArmPositions(getElbowPosition(), getRelativeWristPosition());
  }

  @Override
  public void simulationPeriodic() {
    elbowSim.setInputVoltage(elbowLead.getAppliedOutput());
    elbowSim.update(Constants.RATE);
    elbowEncoder.setPosition(elbowSim.getAngleRads());

    wristSim.setInputVoltage(wrist.getAppliedOutput());
    wristSim.update(Constants.RATE);
    wristEncoder.setPosition(wristSim.getAngleRads());
  }

  @Override
  public void close() {
    wrist.close();
    elbowLead.close();
    elbowFollow.close();
  }
}
