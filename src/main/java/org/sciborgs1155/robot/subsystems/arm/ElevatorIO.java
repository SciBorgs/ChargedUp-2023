package org.sciborgs1155.robot.subsystems.arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public interface ElevatorIO extends Sendable, AutoCloseable {
  public double getHeight();

  public State getState();

  public State getDesiredState();

  public void updateSetpoint(State setpoint);

  public void stopMoving();

  public boolean isFailing();

  public double getVoltage();

  @Override
  default void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("position", () -> getState().position, null);
    builder.addDoubleProperty("velocity", () -> getState().velocity, null);
    builder.addDoubleProperty("target position", () -> getDesiredState().position, null);
    builder.addDoubleProperty("target velocity", () -> getDesiredState().velocity, null);
    builder.addBooleanProperty("failing", this::isFailing, null);
    builder.addDoubleProperty("applied voltage", this::getVoltage, null);
  }
}
