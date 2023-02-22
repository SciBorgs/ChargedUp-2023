package org.sciborgs1155.robot.subsystems;

import static org.sciborgs1155.robot.Ports.Placement.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import org.sciborgs1155.lib.Derivative;
import org.sciborgs1155.lib.PlacementState;
import org.sciborgs1155.lib.Visualizer;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Constants.Arm.ElbowConstants;
import org.sciborgs1155.robot.Constants.Arm.ElevatorConstants;
import org.sciborgs1155.robot.Constants.Arm.WristConstants;
import org.sciborgs1155.robot.subsystems.placement.Elevator;

public class Placement extends SubsystemBase implements Loggable, AutoCloseable {
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

  @Log private final Visualizer visualizer = new Visualizer();

  /** Get current position as a {@link PlacementState} */
  public PlacementState getPosition() {
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
    return getPosition().roughlyEquals(getGoal(), 0.02);
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

  /**
   * Runs elevator, elbow, and wrist to their goals based on a {@link PlacementState}, with the
   * wrist goal relative to the forearm
   */
  public Command runToGoal(PlacementState... goals) {
    Command cmd = Commands.none();
    for (var goal : goals) {
      cmd = cmd.andThen(runToGoal(goal));
    }
    return cmd;
  }

  @Override
  public void periodic() {
    // for now we are using standard feedforward, but as soon as we can test, we will use a
    // feedforward model based on PlacementDynamics

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

    visualizer.setPositions(getPosition());
    visualizer.setSetpoints(getGoal());
  }

  public Command drive(CommandJoystick left, CommandJoystick right) {
    return run(
        () -> {
          double height =
              MathUtil.interpolate(
                  Constants.Dimensions.ELEVATOR_MIN_HEIGHT,
                  Constants.Dimensions.ELEVATOR_MAX_HEIGHT,
                  (left.getZ() - right.getZ()) / 2);
          double elbowAngle =
              MathUtil.interpolate(
                  Constants.Dimensions.ELBOW_MIN_ANGLE,
                  Constants.Dimensions.ELBOW_MAX_ANGLE,
                  (1 - left.getY()) / 2);
          double wristAngle =
              MathUtil.interpolate(
                  Constants.Dimensions.WRIST_MIN_ANGLE,
                  Constants.Dimensions.WRIST_MAX_ANGLE,
                  (1 - right.getY()) / 2);

          elevatorFeedback.setGoal(height);
          elbowFeedback.setGoal(elbowAngle);
          wristFeedback.setGoal(wristAngle);

          System.out.println(elevatorFeedback.getGoal().position);
        });
  }

  @Override
  public void close() {
    elevator.close();
    elbow.close();
    wrist.close();
  }
}
