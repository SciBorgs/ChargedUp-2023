package org.sciborgs1155.robot.subsystems.modules;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import org.sciborgs1155.lib.constants.PIDConstants;
import org.sciborgs1155.robot.Robot;

/** Interface to represent a swerve module */
public interface SwerveModule extends Sendable, AutoCloseable {

  /**
   * Creates a swerve module.
   *
   * <p>If the robot is real, a new {@link MAXSwerveModule} will be created, otherwise either a
   * {@link GoalSwerveModule} or a {@link SimSwerveModule} will be created.
   *
   * <p>For the parameters, see {@link MAXSwerveModule#MAXSwerveModule(int, int, double)}.
   *
   * @return A new {@link SwerveModule} based on if the robot is currently real or simulated.
   */
  public static SwerveModule create(int drivePort, int turnPort, double angularOffset) {
    return Robot.isReal()
        ? new MAXSwerveModule(drivePort, turnPort, angularOffset)
        : new GoalSwerveModule();
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState();

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition();

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState);

  /**
   * Returns the desired state for the module.
   *
   * @return The desired state of the module.
   */
  public SwerveModuleState getDesiredState();

  /** Zeroes all the drive encoders. */
  public void resetEncoders();

  /**
   * Sets the turn PID constants for the module.
   *
   * @param constants Turn PID constants.
   */
  public void setTurnPID(PIDConstants constants);

  /**
   * Sets the drive PID constants for the module.
   *
   * @param constants Drive PID constants.
   */
  public void setDrivePID(PIDConstants constants);

  @Override
  default void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("current velocity", () -> getState().speedMetersPerSecond, null);
    builder.addDoubleProperty("current angle", () -> getPosition().angle.getRadians(), null);
    builder.addDoubleProperty("current position", () -> getPosition().distanceMeters, null);
    builder.addDoubleProperty(
        "target velocity", () -> getDesiredState().speedMetersPerSecond, null);
    builder.addDoubleProperty("target angle", () -> getDesiredState().angle.getRadians(), null);
  }

  @Override
  public void close();
}
