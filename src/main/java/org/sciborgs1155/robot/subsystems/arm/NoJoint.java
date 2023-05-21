package org.sciborgs1155.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

public class NoJoint implements JointIO {

  @Override
  public void close() throws Exception {}

  @Override
  public Rotation2d getRelativeAngle() {
    return new Rotation2d();
  }

  @Override
  public State getCurrentState() {
    return new State();
  }

  @Override
  public State getDesiredState() {
    return new State();
  }

  @Override
  public void updateSetpoint(State setpoint) {}

  @Override
  public void stopMoving() {}

  @Override
  public void setBaseAngle(Rotation2d baseAngle) {}

  @Override
  public Rotation2d getBaseAngle() {
    return new Rotation2d();
  }

  @Override
  public boolean isFailing() {
    return false;
  }

  @Override
  public double getVoltage() {
    return 0;
  }
}
