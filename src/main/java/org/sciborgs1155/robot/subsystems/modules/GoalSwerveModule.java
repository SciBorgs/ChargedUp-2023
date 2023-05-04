package org.sciborgs1155.robot.subsystems.modules;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.sciborgs1155.lib.constants.PIDConstants;
import org.sciborgs1155.robot.Constants;

/** Ideal swerve module, useful for debugging */
public class GoalSwerveModule implements SwerveModule {

  private SwerveModuleState state = new SwerveModuleState();
  private double distance;

  @Override
  public SwerveModuleState getState() {
    return state;
  }

  @Override
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(distance, state.angle);
  }

  @Override
  public void setDesiredState(SwerveModuleState desiredState) {
    state = SwerveModuleState.optimize(desiredState, state.angle);
    distance += state.speedMetersPerSecond * Constants.PERIOD;
  }

  @Override
  public SwerveModuleState getDesiredState() {
    return state;
  }

  @Override
  public void resetEncoders() {}

  @Override
  public void setTurnPID(PIDConstants constants) {}

  @Override
  public void setDrivePID(PIDConstants constants) {}

  @Override
  public void close() {}
}
