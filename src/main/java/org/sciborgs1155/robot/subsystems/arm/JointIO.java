package org.sciborgs1155.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import org.sciborgs1155.lib.constants.ArmFFConstants;
import org.sciborgs1155.lib.constants.PIDConstants;

/**
 * Represents a joint with closed loop control. Note that either {@link this#updateSetpoint(State)}
 * or {@link this#stopMoving()} must be called periodically.
 */
public interface JointIO extends Sendable, AutoCloseable {

  /** A record to store configuration values for a joint */
  public static record JointConfig(
      ArmFFConstants ff,
      PIDConstants pid,
      DCMotor gearbox,
      double gearing,
      double length,
      double mass,
      double minAngle,
      double maxAngle) {}

  /** Returns the angle of the joint relative to the joint it's mounted on */
  public Rotation2d getRelativeAngle();

  /**
   * Returns the state (radians, radians / second) of the joint relative to the joint it's mounted
   * on
   */
  public State getCurrentState();

  /**
   * Returns the target state (radians, radians / second) of the joint relative to the joint it's
   * mounted on
   */
  public State getDesiredState();

  /**
   * Sets the setpoint to the specified state (radians, radians / second) relative to the joint this
   * joint is mounted on and updates the internal closed loop control
   */
  public void updateSetpoint(State setpoint);

  /** Stops the joint from moving by applying a voltage of 0 */
  public void stopMoving();

  /** Sets the angle of the joint that this joint is mounted upon. Used for feedforward. */
  public void setBaseAngle(Rotation2d baseAngle);

  /**
   * Returns the angle of the joint that this joint is mounted upon as a rotation, set with {@link
   * this#setBaseAngle(Rotation2d)}.
   */
  public Rotation2d getBaseAngle();

  /** Returns if the joint is inoperable. */
  public boolean isFailing();

  /** Returns the last applied voltage. */
  public double getVoltage();

  /**
   * Returns the angle of the joint relative to the horizontal, based on {@link
   * this#setBaseAngle(Rotation2d)}.
   */
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
