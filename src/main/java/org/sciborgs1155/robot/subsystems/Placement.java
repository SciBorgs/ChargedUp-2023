package org.sciborgs1155.robot.subsystems;

import static org.sciborgs1155.robot.Ports.Placement.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import org.sciborgs1155.lib.Derivative;
import org.sciborgs1155.lib.PlacementState;
import org.sciborgs1155.robot.Constants.Arm.ElbowConstants;
import org.sciborgs1155.robot.Constants.Arm.ElevatorConstants;
import org.sciborgs1155.robot.Constants.Arm.WristConstants;
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

  @Log(name = "Elevator Acceleration", methodName = "getLastOutput")
  private final Derivative elevatorAccel = new Derivative();

  @Log(name = "Wrist Acceleration", methodName = "getLastOutput")
  private final Derivative wristAccel = new Derivative();

  @Log(name = "Elbow Acceleration", methodName = "getLastOutput")
  private final Derivative elbowAccel = new Derivative();

  /** Get current position as a {@link PlacementState} */
  public PlacementState getState() {
    return new PlacementState(
        elevator.getPosition(),
        elbow.getPosition(),
        wrist.getPosition(),
        elevator.getVelocity(),
        elbow.getVelocity(),
        wrist.getVelocity());
  }

  /** Get current goal as a {@link PlacementState} */
  public PlacementState getGoal() {
    return new PlacementState(
        elevatorFeedback.getGoal().position,
        Rotation2d.fromRadians(elbowFeedback.getGoal().position),
        Rotation2d.fromRadians(wristFeedback.getGoal().position),
        elevatorFeedback.getGoal().velocity,
        elbowFeedback.getGoal().velocity,
        wristFeedback.getGoal().velocity);
  }

  @Log(name = "At Goal")
  public boolean atGoal() {
    return getState().roughlyEquals(getGoal(), 0.02);
  }

  /**
   * Sets elevator, elbow, and wrist goals based on a {@link PlacementState}, with the wrist goal
   * relative to the forearm
   */
  public Command setGoal(PlacementState goal) {
    return runOnce(
        () -> {
          elevatorFeedback.setGoal(goal.elevatorState());
          elbowFeedback.setGoal(goal.elbowState());
          wristFeedback.setGoal(goal.wristState());
        });
  }

  /**
   * Runs elevator, elbow, and wrist to their goals based on a {@link PlacementState}, with the
   * wrist goal relative to the forearm
   */
  public Command runToGoal(PlacementState goal) {
    return setGoal(goal).andThen(Commands.waitUntil(this::atGoal));
  }

  @Override
  public void periodic() {
    double elevatorFB = elevatorFeedback.calculate(elevator.getPosition());
    double elevatorFF =
        elevatorFeedforward.calculate(
            elevatorFeedback.getSetpoint().velocity,
            elevatorAccel.calculate(elevatorFeedback.getSetpoint().velocity));

    elevator.setVoltage(elevatorFB + elevatorFF);

    double elbowFB = elbowFeedback.calculate(elbow.getPosition().getRadians());
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
    double wristFB = wristFeedback.calculate(wrist.getPosition().getRadians());
    double wristFF =
        wristFeedforward.calculate(
            wristFeedback.getSetpoint().position + elbowFeedback.getSetpoint().position,
            wristFeedback.getSetpoint().velocity,
            wristAccel.calculate(wristFeedback.getSetpoint().velocity));
    wrist.setVoltage(wristFB + wristFF);
  }

  @Override
  public void close() {
    elevator.close();
    elbow.close();
    wrist.close();
  }
}
