package org.sciborgs1155.robot.subsystems;

import static org.sciborgs1155.robot.Constants.ArmConstants.*;
import static org.sciborgs1155.robot.Ports.Arm.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import org.sciborgs1155.lib.Derivative;
import org.sciborgs1155.lib.Visualizer;
import org.sciborgs1155.robot.Constants.Dimensions;
import org.sciborgs1155.robot.subsystems.joints.Elbow;
import org.sciborgs1155.robot.subsystems.joints.Wrist;

public class Arm extends SubsystemBase implements Loggable, AutoCloseable {

  @Log private final Wrist wrist = Wrist.create(WRIST_MOTOR);

  @Log private final Elbow elbow = Elbow.create(LEFT_ELBOW_MOTOR, RIGHT_ELBOW_MOTOR);

  @Log
  private final ProfiledPIDController wristFeedback =
      new ProfiledPIDController(
          WristConstants.kP, WristConstants.kI, WristConstants.kD, WristConstants.CONSTRAINTS);

  @Log
  private final ProfiledPIDController elbowFeedback =
      new ProfiledPIDController(
          ElbowConstants.kP, ElbowConstants.kI, ElbowConstants.kD, ElbowConstants.CONSTRAINTS);

  private final ArmFeedforward wristFeedforward =
      new ArmFeedforward(
          WristConstants.kS, WristConstants.kG, WristConstants.kV, WristConstants.kA);

  private final ArmFeedforward elbowFeedforward =
      new ArmFeedforward(
          ElbowConstants.kS, ElbowConstants.kG, ElbowConstants.kV, ElbowConstants.kA);

  // private final SimpleMotorFeedforward wristFeedforward = new
  // SimpleMotorFeedforward(WristConstants.kS, WristConstants.kV, WristConstants.kA);

  // private final SimpleMotorFeedforward elbowFeedforward = new
  // SimpleMotorFeedforward(ElbowConstants.kS, ElbowConstants.kV, ElbowConstants.kA);

  @Log(name = "wrist acceleration", methodName = "getLastOutput")
  private final Derivative wristAccel = new Derivative();

  @Log(name = "elbow acceleration", methodName = "getLastOutput")
  private final Derivative elbowAccel = new Derivative();

  private final Visualizer visualizer;

  public Arm(Visualizer visualizer) {
    this.visualizer = visualizer;
  }

  /** Elbow position relative to the chassis */
  public Rotation2d getElbowPosition() {
    return elbow.getPosition();
  }

  /** Wrist position relative to the forearm */
  public Rotation2d getRelativeWristPosition() {
    return wrist.getPosition();
  }

  /** Wrist position relative to chassis */
  @Log(name = "wrist absolute positon", methodName = "getDegrees")
  public Rotation2d getAbsoluteWristPosition() {
    return getRelativeWristPosition().plus(getElbowPosition());
  }

  /** Elbow goal relative to the chassis */
  public Rotation2d getElbowGoal() {
    return Rotation2d.fromRadians(elbowFeedback.getGoal().position);
  }

  /** Wrist goal relative to forearm */
  public Rotation2d getRelativeWristGoal() {
    return Rotation2d.fromRadians(wristFeedback.getGoal().position);
  }

  /** Wrist goal relative to the chassis */
  public Rotation2d getAbsoluteWristGoal() {
    return getRelativeWristGoal().plus(getElbowGoal());
  }

  /** Sets elbow goal relative to the chassis */
  public Command setElbowGoal(Rotation2d goal) {
    return runOnce(
        () ->
            elbowFeedback.setGoal(
                MathUtil.clamp(
                    goal.getRadians(), Dimensions.ELBOW_MIN_ANGLE, Dimensions.ELBOW_MAX_ANGLE)));
  }

  /** Sets wrist goal relative to the forearm */
  public Command setWristGoal(Rotation2d goal) {
    return runOnce(
        () ->
            wristFeedback.setGoal(
                MathUtil.clamp(
                    goal.getRadians(), Dimensions.WRIST_MIN_ANGLE, Dimensions.WRIST_MAX_ANGLE)));
  }

  /** Sets elbow and wrist goals, with the wrist goal relative to the forearm */
  public Command setGoals(Rotation2d elbowGoal, Rotation2d wristGoal) {
    return setElbowGoal(elbowGoal).andThen(setWristGoal(wristGoal));
  }

  /** Runs elbow to goal relative to the chassis */
  public Command runElbowToGoal(Rotation2d goal) {
    return setElbowGoal(goal).andThen(Commands.waitUntil(elbowFeedback::atGoal));
  }

  /** Runs wrist to goal relative to the forearm */
  public Command runWristToGoal(Rotation2d goal) {
    return setWristGoal(goal).andThen(Commands.waitUntil(wristFeedback::atGoal));
  }

  /** Runs elbow and wrist go provided goals, with the wrist goal relative to the forearm */
  public Command runToGoals(Rotation2d elbowGoal, Rotation2d wristGoal) {
    return setGoals(elbowGoal, wristGoal)
        .andThen(Commands.waitUntil(() -> elbowFeedback.atGoal() && wristFeedback.atGoal()));
  }

  @Override
  public void periodic() {
    double elbowfb = elbowFeedback.calculate(elbow.getPosition().getRadians());
    double elbowff =
        elbowFeedforward.calculate(
            elbowFeedback.getSetpoint().position,
            elbowFeedback.getSetpoint().velocity,
            elbowAccel.calculate(elbowFeedback.getSetpoint().velocity));
    elbow.setVoltage(elbowfb + elbowff);

    // wrist feedback is calculated using an absolute angle setpoint, rather than a relative one
    // this means the extra voltage calculated to cancel out gravity is kG * cos(θ + ϕ), where θ is
    // the elbow setpoint and ϕ is the wrist setpoint
    // the elbow angle is used as a setpoint instead of current position because we're using a
    // profiled pid controller, which means setpoints are achievable states, rather than goals
    double wristfb = wristFeedback.calculate(wrist.getPosition().getRadians());
    double wristff =
        wristFeedforward.calculate(
            wristFeedback.getSetpoint().position + elbowFeedback.getSetpoint().position,
            wristFeedback.getSetpoint().velocity,
            wristAccel.calculate(wristFeedback.getSetpoint().velocity));
    wrist.setVoltage(wristfb + wristff);

    visualizer.setArmPositions(getElbowPosition(), getRelativeWristPosition());
  }

  @Override
  public void close() {
    // wrist.close();
    // elbow.close();
  }
}
