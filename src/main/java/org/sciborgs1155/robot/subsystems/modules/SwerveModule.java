package org.sciborgs1155.robot.subsystems.modules;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.sciborgs1155.robot.Robot;

public interface SwerveModule {

  public static SwerveModule create(int drivePort, int turnPort, double angularOffset) {
    return Robot.isReal()
        ? new MAXSwerveModule(drivePort, turnPort, angularOffset)
        : new SimSwerveModule();
  }

  public SwerveModuleState getState();

  public SwerveModulePosition getPosition();

  public void setDesiredState(SwerveModuleState desiredState);

  public void resetEncoders();
}
