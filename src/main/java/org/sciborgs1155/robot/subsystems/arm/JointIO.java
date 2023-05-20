package org.sciborgs1155.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public interface JointIO extends Sendable, AutoCloseable {
  public Rotation2d getRelativeAngle();

  public State getCurrentState();

  public State getDesiredState();

  public void update(State setpoint);

  public void setBaseAngle(Rotation2d baseAngle);

  public Rotation2d getBaseAngle();

  public boolean isFailing();

  public double getVoltage();

  public default Rotation2d getAbsoluteAngle() {
    return getBaseAngle().plus(getRelativeAngle());
  }

  @Override
  default void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("current angle", () -> getCurrentState().position, null);
    builder.addDoubleProperty("current angular velocity", () -> getCurrentState().velocity, null);
    builder.addDoubleProperty("target angle", () -> getDesiredState().position, null);
    builder.addDoubleProperty("target angular velocity", () -> getDesiredState().velocity, null);
    builder.addDoubleProperty("base angle", () -> getBaseAngle().getRadians(), null);
    builder.addDoubleProperty("absolute angle", () -> getAbsoluteAngle().getRadians(), null);
    builder.addBooleanProperty("failing", this::isFailing, null);
    builder.addDoubleProperty("applied voltage", this::getVoltage, null);
  }
}
