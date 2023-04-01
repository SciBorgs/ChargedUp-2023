package org.sciborgs1155.robot.subsystems;

import static org.sciborgs1155.robot.Constants.Arm.*;
import static org.sciborgs1155.robot.Ports.Arm.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import org.sciborgs1155.lib.Derivative;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Constants.Dimensions;
import org.sciborgs1155.robot.Robot;
import org.sciborgs1155.robot.util.Visualizer;

public class Arm extends SubsystemBase implements Loggable, AutoCloseable {

  @Log(name = "elbow applied output", methodName = "getAppliedOutput")
  private final CANSparkMax elbow = Elbow.MOTOR.build(MotorType.kBrushless, MIDDLE_ELBOW_MOTOR);

  private final CANSparkMax elbowLeft = Elbow.MOTOR.build(MotorType.kBrushless, LEFT_ELBOW_MOTOR);
  private final CANSparkMax elbowRight = Elbow.MOTOR.build(MotorType.kBrushless, RIGHT_ELBOW_MOTOR);

  @Log(name = "wrist applied output", methodName = "getAppliedOutput")
  private final CANSparkMax wrist = Wrist.MOTOR.build(MotorType.kBrushless, WRIST_MOTOR);

  private final RelativeEncoder elbowEncoder = elbow.getAlternateEncoder(8192);

  @Log(name = "wrist velocity", methodName = "getVelocity")
  private final AbsoluteEncoder wristEncoder = wrist.getAbsoluteEncoder(Type.kDutyCycle);

  private final ArmFeedforward elbowFeedforward =
      new ArmFeedforward(Elbow.FF.s(), Elbow.FF.g(), Elbow.FF.v(), Elbow.FF.a());
  private final ArmFeedforward wristFeedforward =
      new ArmFeedforward(Wrist.FF.s(), Wrist.FF.g(), Wrist.FF.v(), Wrist.FF.a());

  @Log(name = "elbow feedback")
  @Log(name = "elbow at goal", methodName = "atGoal")
  private final ProfiledPIDController elbowFeedback =
      new ProfiledPIDController(Elbow.PID.p(), Elbow.PID.i(), Elbow.PID.d(), Elbow.CONSTRAINTS);

  @Log(name = "wrist feedback")
  @Log(name = "wrist at goal", methodName = "atGoal")
  private final ProfiledPIDController wristFeedback =
      new ProfiledPIDController(Wrist.PID.p(), Wrist.PID.i(), Wrist.PID.d(), Wrist.CONSTRAINTS);

  @Log(name = "wrist acceleration", methodName = "getLastOutput")
  private final Derivative wristAccel = new Derivative();

  @Log(name = "elbow acceleration", methodName = "getLastOutput")
  private final Derivative elbowAccel = new Derivative();

  private final SingleJointedArmSim elbowSim =
      new SingleJointedArmSim(
          DCMotor.getNEO(3),
          1 / Elbow.CONVERSION.gearing(),
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

    elbowEncoder.setPosition(Elbow.ELBOW_OFFSET);
    elbowEncoder.setPositionConversionFactor(Elbow.CONVERSION.factor());
    elbowEncoder.setVelocityConversionFactor(Elbow.CONVERSION.factor() / 60.0);
    wristEncoder.setPositionConversionFactor(Wrist.CONVERSION.factor());
    wristEncoder.setVelocityConversionFactor(Wrist.CONVERSION.factor() / 60.0);

    // set wrist duty cycle absolute encoder frame periods to be the same as our tickrate
    // periodic frames 3 and 4 are useless to us, so to improve performance we set them to 1155 ms
    wrist.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1155);
    wrist.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1155);
    wrist.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
    wrist.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);

    elbow.burnFlash();
    elbowLeft.burnFlash();
    elbowRight.burnFlash();
    wrist.burnFlash();

    this.visualizer = visualizer;

    elbowFeedback.setGoal(getElbowPosition().getRadians());
    wristFeedback.setGoal(Math.PI);
    wristFeedback.setTolerance(0.3);
  }

  /** Elbow position relative to the chassis */
  @Log(name = "elbow position", methodName = "getRadians")
  public Rotation2d getElbowPosition() {
    return Rotation2d.fromRadians(elbowEncoder.getPosition());
  }

  /** Wrist position relative to the forearm */
  @Log(name = "relative wrist position", methodName = "getRadians")
  public Rotation2d getRelativeWristPosition() {
    // encoder is zeroed fully folded in, which is actually PI, so we offset by -PI
    return Rotation2d.fromRadians(
        Robot.isReal() ? wristEncoder.getPosition() - Math.PI : wristSim.getAngleRads());
  }

  /** Wrist position relative to chassis */
  @Log(name = "absolute wrist position", methodName = "getRadians")
  public Rotation2d getAbsoluteWristPosition() {
    return getRelativeWristPosition().plus(getElbowPosition());
  }

  /** Elbow and wrist at goals */
  public boolean atGoal() {
    return elbowFeedback.atGoal() && wristFeedback.atGoal();
  }

  /** Sets elbow goal relative to the chassis */
  public Command setElbowGoal(State goal) {
    return runOnce(
        () ->
            elbowFeedback.setGoal(
                new State(
                    MathUtil.clamp(
                        goal.position, Dimensions.ELBOW_MIN_ANGLE, Dimensions.ELBOW_MAX_ANGLE),
                    goal.velocity)));
  }

  /** Sets wrist goal relative to being parallel to the forearm */
  public Command setWristGoal(State goal) {
    return runOnce(
        () ->
            wristFeedback.setGoal(
                new State(
                    MathUtil.clamp(
                        goal.position, Dimensions.WRIST_MIN_ANGLE, Dimensions.WRIST_MAX_ANGLE),
                    goal.velocity)));
  }

  /** Sets both the wrist and elbow goals */
  public Command setGoals(Rotation2d elbowGoal, Rotation2d wristGoal) {
    return setGoals(new State(elbowGoal.getRadians(), 0), new State(wristGoal.getRadians(), 0));
  }

  /** Sets both the wrist and elbow goals */
  public Command setGoals(State elbowGoal, State wristGoal) {
    return setElbowGoal(elbowGoal).andThen(setWristGoal(wristGoal));
  }

  /** Runs elbow to goal relative to the chassis */
  public Command runElbowToGoal(State goal) {
    return setElbowGoal(goal).andThen(Commands.waitUntil(elbowFeedback::atGoal));
  }

  /** Runs wrist to goal relative to the forearm */
  public Command runWristToGoal(State goal) {
    return setWristGoal(goal).andThen(Commands.waitUntil(wristFeedback::atGoal));
  }

  /** Runs elbow and wrist go provided goals, with the wrist goal relative to the forearm */
  public Command runToGoals(State elbowGoal, State wristGoal) {
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
        getElbowPosition(), Rotation2d.fromRadians(elbowFeedback.getSetpoint().position));
    visualizer.setWrist(
        getRelativeWristPosition(), Rotation2d.fromRadians(wristFeedback.getSetpoint().position));
    System.out.println("elbow pos: " + getElbowPosition().getRadians());
  }

  @Override
  public void simulationPeriodic() {
    elbowSim.setInputVoltage(elbow.getAppliedOutput());
    elbowSim.update(Constants.RATE);
    elbowEncoder.setPosition(elbowSim.getAngleRads());

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
