package org.sciborgs1155.robot.subsystems.modules;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.sciborgs1155.lib.Kinematics.SciSwerveModuleState;
import org.sciborgs1155.robot.Robot;

public interface SwerveModule {

  public static SwerveModule create(int drivePort, int turnPort, double angularOffset) {
    return Robot.isReal()
        ? new MAXSwerveModule(drivePort, turnPort, angularOffset)
        : new SimSwerveModule();
  }

  public SciSwerveModuleState getState();

  public SwerveModuleState getAutoState();

  public SwerveModulePosition getPosition();

  public void setDesiredState(SciSwerveModuleState desiredStates);

  public void setDesiredState(SwerveModuleState desiredStates);

  public void resetEncoders();
}
