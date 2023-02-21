package org.sciborgs1155.robot.subsystems;

import static org.sciborgs1155.robot.Ports.Placement.*;

import org.sciborgs1155.lib.PlacementState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import org.sciborgs1155.robot.Constants.Arm.ElbowConstants;
import org.sciborgs1155.robot.Constants.Arm.ElevatorConstants;
import org.sciborgs1155.robot.Constants.Arm.WristConstants;
import org.sciborgs1155.robot.Constants.Dimensions;
import org.sciborgs1155.robot.subsystems.placement.Elbow;
import org.sciborgs1155.robot.subsystems.placement.Elevator;
import org.sciborgs1155.robot.subsystems.placement.Wrist;

public class Placement extends SubsystemBase implements Loggable, AutoCloseable {
  @Log private final Wrist wrist = Wrist.create(WRIST_MOTOR);

  @Log
  private final Elbow elbow = Elbow.create(MIDDLE_ELBOW_MOTOR, LEFT_ELBOW_MOTOR, RIGHT_ELBOW_MOTOR);

  @Log
  private final Elevator elevator =
      Elevator.create(MIDDLE_ELEVATOR_MOTOR, LEFT_ELEVATOR_MOTOR, RIGHT_ELEVATOR_MOTOR);

  @Log(name = "Elevator Feedback")
  private final ProfiledPIDController elevatorFeedback =
      new ProfiledPIDController(
          ElevatorConstants.kP,
          ElevatorConstants.kI,
          ElevatorConstants.kD,
          ElevatorConstants.CONSTRAINTS);

  @Log(name = "Elbow Feedback")
  private final ProfiledPIDController elbowFeedback =
      new ProfiledPIDController(
          ElbowConstants.kP, ElbowConstants.kI, ElbowConstants.kD, ElbowConstants.CONSTRAINTS);

  @Log(name = "Wrist Feedback")
  private final ProfiledPIDController wristFeedback =
      new ProfiledPIDController(
          WristConstants.kP, WristConstants.kI, WristConstants.kD, WristConstants.CONSTRAINTS);

  private final ElevatorFeedforward elevatorFeedforward =
      new ElevatorFeedforward(
          ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV, ElevatorConstants.kA);

  private final ArmFeedforward wristFeedforward =
      new ArmFeedforward(
          WristConstants.kS, WristConstants.kG, WristConstants.kV, WristConstants.kA);

  private final ArmFeedforward elbowFeedforward =
      new ArmFeedforward(
          ElbowConstants.kS, ElbowConstants.kG, ElbowConstants.kV, ElbowConstants.kA);

  @Log(name = "Last Elevator Velocity")
  private double lastElevatorVelocity;

  @Log(name = "Last Elbow Velocity")
  private double lastElbowVelocity;

  @Log(name = "Last Wrist Velocity")
  private double lastWristVelocity;

  /** Get current state */
  public PlacementState getState() {
    return new PlacementState(elevator.getPosition(), elbow.getPosition(), wrist.getPosition(), elevator.getVelocity(), elbow.getVelocity(), wrist.getVelocity());
  }

  /** Get current setpoint */
  public PlacementState getSetpoint() {
    return new PlacementState(elevatorFeedback.getGoal().position, elbowFeedback.getGoal(), getAbsoluteWristGoal(), MIDDLE_ELBOW_MOTOR, LEFT_ELEVATOR_MOTOR, LEFT_ELBOW_MOTOR);
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

  /** Elevator goal from the base, in meters */
  public Command setGoal(double height) {
    return runOnce(
        () ->
            elevatorFeedback.setGoal(
                MathUtil.clamp(
                    height, Dimensions.ELEVATOR_MIN_HEIGHT, Dimensions.ELEVATOR_MAX_HEIGHT)));
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
}
