package org.sciborgs1155.robot.subsystems.modules;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import org.sciborgs1155.robot.Robot;

public interface SwerveModule extends Sendable {

  public static SwerveModule create(int drivePort, int turnPort, double angularOffset) {
    return Robot.isReal()
        ? new MAXSwerveModule(drivePort, turnPort, angularOffset)
        : new SimSwerveModule();
  }

  public SwerveModuleState getState();

  public SwerveModulePosition getPosition();

  public void setDesiredState(SwerveModuleState desiredState);

  public SwerveModuleState getDesiredState();

  public void resetEncoders();

  @Override
  default void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("current velocity", () -> getState().speedMetersPerSecond, null);
    builder.addDoubleProperty("current angle", () -> getPosition().angle.getRadians(), null);
    builder.addDoubleProperty("current position", () -> getPosition().distanceMeters, null);
    builder.addDoubleProperty(
        "target velocity", () -> getDesiredState().speedMetersPerSecond, null);
    builder.addDoubleProperty("target angle", () -> getDesiredState().angle.getRadians(), null);
  }
}
