package org.sciborgs1155.robot.subsystems.arm;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import org.sciborgs1155.lib.failure.Fallible;

/**
 * Represents an elevator with closed loop control. Note that either {@link
 * this#updateSetpoint(State)} or {@link this#stopMoving()} must be called periodically.
 */
public interface ElevatorIO extends Sendable, Fallible, AutoCloseable {

  /** A record to store configuration values for an elevator */
  public static record ElevatorConfig(
      ElevatorFFConstants ff,
      PIDConstants pid,
      DCMotor gearbox,
      double gearing,
      double mass,
      double sprocketRadius,
      double minHeight,
      double maxHeight) {}

  /** Returns the height of the elevator from the ground. */
  public double getHeight();

  /** Returns the current state (m, m/s) of the elevator. */
  public State getCurrentState();

  /** Returns the desired state (m, m/s) of the elevator. */
  public State getDesiredState();

  /**
   * Sets the setpoint to the specified state (m, m/s) of the elevator and updates the internal
   * closed loop control.
   */
  public void updateSetpoint(State setpoint);

  /** Stops the elevator from moving by applying a voltage of 0. */
  public void stopMoving();

  /** Returns the last applied voltage. */
  public double getVoltage();

  @Override
  default void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("position", () -> getCurrentState().position, null);
    builder.addDoubleProperty("velocity", () -> getCurrentState().velocity, null);
    builder.addDoubleProperty("target position", () -> getDesiredState().position, null);
    builder.addDoubleProperty("target velocity", () -> getDesiredState().velocity, null);
    builder.addBooleanProperty("failing", this::isFailing, null);
    builder.addDoubleProperty("applied voltage", this::getVoltage, null);
  }
}
