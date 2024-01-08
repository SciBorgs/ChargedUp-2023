package org.sciborgs1155.robot.arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import java.util.List;
import org.sciborgs1155.lib.failure.HardwareFault;

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
  public double getVoltage() {
    return 0;
  }

  @Override
  public List<HardwareFault> getFaults() {
    return List.of();
  }
}
