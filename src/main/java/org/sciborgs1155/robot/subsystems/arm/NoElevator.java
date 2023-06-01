package org.sciborgs1155.robot.subsystems.arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

public class NoElevator implements ElevatorIO {

  @Override
  public void close() throws Exception {}

  @Override
  public double getHeight() {
    return 0;
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
  public boolean isFailing() {
    return false;
  }

  @Override
  public double getVoltage() {
    return 0;
  }
}
