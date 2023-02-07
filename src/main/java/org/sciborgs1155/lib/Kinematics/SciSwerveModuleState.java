package org.sciborgs1155.lib.Kinematics;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SciSwerveModuleState extends SwerveModuleState {
  /** IN METERS PER SECOND SQUARED */
  public double moduleAcceleration;

  public Rotation2d angle;

  /** Create an empty ModuleState with no accel or angle */
  public SciSwerveModuleState() {}

  public SciSwerveModuleState(double moduleAcceleration, double velocity, Rotation2d angle) {
    this.moduleAcceleration = moduleAcceleration;
    this.angle = angle;
    speedMetersPerSecond = velocity;
  }

  @Override
  public int compareTo(SwerveModuleState arg0) {
    // TODO Auto-generated method stub
    return 0;
  }

  public static SciSwerveModuleState optimize(
      SciSwerveModuleState desiredState, Rotation2d currentAngle) {
    var delta = desiredState.angle.minus(currentAngle);
    if (Math.abs(delta.getDegrees()) > 90.0) {
      return new SciSwerveModuleState(
          -desiredState.moduleAcceleration,
          -desiredState.speedMetersPerSecond,
          desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0)));
    } else {
      return new SciSwerveModuleState(
          desiredState.moduleAcceleration, desiredState.speedMetersPerSecond, desiredState.angle);
    }
  }
}
