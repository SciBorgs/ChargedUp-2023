package org.sciborgs1155.robot.subsystems;

import static org.sciborgs1155.robot.Ports.Arm.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import org.sciborgs1155.lib.Derivative;
import org.sciborgs1155.lib.Visualizer;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Constants.Arm.Elbow;
import org.sciborgs1155.robot.Constants.Arm.Wrist;
import org.sciborgs1155.robot.Constants.Dimensions;
import org.sciborgs1155.robot.Constants.Motors;
import org.sciborgs1155.robot.Robot;

public class Arm extends SubsystemBase implements Loggable, AutoCloseable {

  @Log(name = "elbow applied output", methodName = "getAppliedOutput")
  private final CANSparkMax elbow = Motors.ELBOW.build(MotorType.kBrushless, MIDDLE_ELBOW_MOTOR);

  private final CANSparkMax elbowLeft = Motors.ELBOW.build(MotorType.kBrushless, LEFT_ELBOW_MOTOR);
  private final CANSparkMax elbowRight =
      Motors.ELBOW.build(MotorType.kBrushless, RIGHT_ELBOW_MOTOR);

  @Log(name = "wrist applied output", methodName = "getAppliedOutput")
  private final CANSparkMax wrist = Motors.WRIST.build(MotorType.kBrushless, WRIST_MOTOR);

  @Log private final Encoder elbowEncoder = new Encoder(ELBOW_ENCODER[0], ELBOW_ENCODER[1]);
  private final EncoderSim elbowEncoderSim = new EncoderSim(elbowEncoder);

  @Log(name = "wrist velocity", methodName = "getVelocity")
  @Log(name = "Wrist Position!!!", methodName = "getPosition")
  private final RelativeEncoder wristEncoder = wrist.getEncoder();

  private final ArmFeedforward elbowFeedforward =
      new ArmFeedforward(Elbow.kS, Elbow.kG, Elbow.kV, Elbow.kA);
  private final ArmFeedforward wristFeedforward =
      new ArmFeedforward(Wrist.kS, Wrist.kG, Wrist.kV, Wrist.kA);

  @Log(name = "elbow feedback")
  private final ProfiledPIDController elbowFeedback =
      new ProfiledPIDController(Elbow.kP, Elbow.kI, Elbow.kD, Elbow.CONSTRAINTS);

  @Log(name = "wrist feedback")
  private final ProfiledPIDController wristFeedback =
      new ProfiledPIDController(Wrist.kP, Wrist.kI, Wrist.kD, Wrist.CONSTRAINTS);

  @Log(name = "wrist acceleration", methodName = "getLastOutput")
  private final Derivative wristAccel = new Derivative();

  @Log(name = "elbow acceleration", methodName = "getLastOutput")
  private final Derivative elbowAccel = new Derivative();

  private final SingleJointedArmSim elbowSim =
      new SingleJointedArmSim(
          DCMotor.getNEO(3),
          1 / Elbow.CONVERSION,
          SingleJointedArmSim.estimateMOI(Dimensions.FOREARM_LENGTH, Dimensions.FOREARM_MASS),
          Dimensions.FOREARM_LENGTH,
          Dimensions.ELBOW_MIN_ANGLE,
          Dimensions.ELBOW_MAX_ANGLE,
          true);
  private final SingleJointedArmSim wristSim =
      new SingleJointedArmSim(
          DCMotor.getNEO(1),
          1,
          SingleJointedArmSim.estimateMOI(Dimensions.CLAW_LENGTH, Dimensions.CLAW_MASS),
          Dimensions.CLAW_LENGTH,
          Dimensions.WRIST_MIN_ANGLE,
          Dimensions.WRIST_MAX_ANGLE,
          false);

  private final Visualizer visualizer;

  public Arm(Visualizer visualizer) {
    elbowLeft.follow(elbow);
    elbowRight.follow(elbow);

    elbowEncoder.setDistancePerPulse(Elbow.ENCODER_FACTOR);
    wristEncoder.setPositionConversionFactor(1 / 47.2);

    elbow.burnFlash();
    elbowLeft.burnFlash();
    elbowRight.burnFlash();
    wrist.burnFlash();

    this.visualizer = visualizer;
  }

  /** Elbow position relative to the chassis */
  public Rotation2d getElbowPosition() {
    return Rotation2d.fromRadians(elbowEncoder.getDistance());
  }

  /** Wrist position relative to the forearm */
  public Rotation2d getRelativeWristPosition() {
    return Rotation2d.fromRadians(
        Robot.isReal() ? wristEncoder.getPosition() : wristSim.getAngleRads());
  }

  /** Wrist position relative to chassis */
  public Rotation2d getAbsoluteWristPosition() {
    return getRelativeWristPosition().plus(getElbowPosition());
  }

  /** Elbow and wrist at goals */
  public boolean atGoal() {
    return elbowFeedback.atGoal() && wristFeedback.atGoal();
  }

  /** Sets elbow goal relative to the chassis */
  public Command setElbowGoal(TrapezoidProfile.State goal) {
    return runOnce(
        () ->
            elbowFeedback.setGoal(
                new TrapezoidProfile.State(
                    MathUtil.clamp(
                        goal.position, Dimensions.ELBOW_MIN_ANGLE, Dimensions.ELBOW_MAX_ANGLE),
                    goal.velocity)));
  }

  /** Sets wrist goal relative to the forearm */
  public Command setWristGoal(TrapezoidProfile.State goal) {
    return runOnce(
        () ->
            wristFeedback.setGoal(
                new TrapezoidProfile.State(
                    MathUtil.clamp(
                        goal.position, Dimensions.WRIST_MIN_ANGLE, Dimensions.WRIST_MAX_ANGLE),
                    goal.velocity)));
  }

  /** Sets both the wrist and elbow goals */
  public Command setGoals(TrapezoidProfile.State elbowGoal, TrapezoidProfile.State wristGoal) {
    return setElbowGoal(elbowGoal).andThen(setWristGoal(wristGoal));
  }

  /** Runs elbow to goal relative to the chassis */
  public Command runElbowToGoal(TrapezoidProfile.State goal) {
    return setElbowGoal(goal).andThen(Commands.waitUntil(elbowFeedback::atGoal));
  }

  /** Runs wrist to goal relative to the forearm */
  public Command runWristToGoal(TrapezoidProfile.State goal) {
    return setWristGoal(goal).andThen(Commands.waitUntil(wristFeedback::atGoal));
  }

  /** Runs elbow and wrist go provided goals, with the wrist goal relative to the forearm */
  public Command runToGoals(TrapezoidProfile.State elbowGoal, TrapezoidProfile.State wristGoal) {
    return setGoals(elbowGoal, wristGoal).andThen(Commands.waitUntil(this::atGoal));
  }

  @Override
  public void periodic() {
    double elbowFB = elbowFeedback.calculate(getElbowPosition().getRadians());
    double elbowFF =
        elbowFeedforward.calculate(
            elbowFeedback.getSetpoint().position,
            elbowFeedback.getSetpoint().velocity,
            elbowAccel.calculate(elbowFeedback.getSetpoint().velocity));
    elbow.setVoltage(elbowFB + elbowFF);

    // wrist feedback is calculated using an absolute angle setpoint, rather than a relative one
    // this means the extra voltage calculated to cancel out gravity is kG * cos(θ + ϕ), where θ is
    // the elbow setpoint and ϕ is the wrist setpoint
    // the elbow angle is used as a setpoint instead of current position because we're using a
    // profiled pid controller, which means setpoints are achievable states, rather than goals
    double wristFB = wristFeedback.calculate(getRelativeWristPosition().getRadians());
    double wristFF =
        wristFeedforward.calculate(
            wristFeedback.getSetpoint().position + elbowFeedback.getSetpoint().position,
            wristFeedback.getSetpoint().velocity,
            wristAccel.calculate(wristFeedback.getSetpoint().velocity));
    wrist.setVoltage(wristFB + wristFF);

    visualizer.setElbow(
        getElbowPosition(), Rotation2d.fromRadians(elbowFeedback.getGoal().position));
    visualizer.setWrist(
        getRelativeWristPosition(), Rotation2d.fromRadians(wristFeedback.getGoal().position));
  }

  @Override
  public void simulationPeriodic() {
    elbowSim.setInputVoltage(elbow.getAppliedOutput());
    elbowSim.update(Constants.RATE);
    elbowEncoderSim.setDistance(elbowSim.getAngleRads());
    elbowEncoderSim.setRate(elbowSim.getVelocityRadPerSec());

    wristSim.setInputVoltage(wrist.getAppliedOutput());
    wristSim.update(Constants.RATE);
  }

  @Override
  public void close() {
    elbow.close();
    elbowLeft.close();
    elbowRight.close();
    wrist.close();
  }
}
