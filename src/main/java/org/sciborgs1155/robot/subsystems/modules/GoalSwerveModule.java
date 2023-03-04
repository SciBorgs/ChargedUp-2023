package org.sciborgs1155.robot.subsystems.modules;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.sciborgs1155.lib.Kinematics.SciSwerveModuleState;
import org.sciborgs1155.robot.Constants;

/** Ideal swerve module, useful for debugging */
public class GoalSwerveModule implements SwerveModule {

  private SwerveModuleState state = new SwerveModuleState();
  private SciSwerveModuleState sciState = new SciSwerveModuleState();
  private double distance;
  private double sciDist;

  @Override
  public SwerveModuleState getAutoState() {
    return state;
  }

  @Override
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(distance, state.angle);
  }

  @Override
  public void setDesiredState(SwerveModuleState desiredState) {
    state = SwerveModuleState.optimize(desiredState, state.angle);
    distance += state.speedMetersPerSecond * Constants.RATE;
  }

  @Override
  public SwerveModuleState getDesiredState() {
    return state;
  }

  @Override
  public void resetEncoders() {}

  @Override
  public SciSwerveModuleState getState() {
    return sciState;
  }

  @Override
  public void setDesiredState(SciSwerveModuleState desiredStates) {
    sciState = SciSwerveModuleState.optimize(desiredStates, sciState.angle);
    sciDist += sciState.speedMetersPerSecond * Constants.RATE;
  }
}
