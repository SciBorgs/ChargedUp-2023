package org.sciborgs1155.robot.subsystems.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import org.sciborgs1155.lib.BetterElevatorFeedforward;
import org.sciborgs1155.robot.Constants;

public class SimElevator implements ElevatorIO {

  private final ElevatorSim sim;

  private final PIDController pid;
  private final BetterElevatorFeedforward ff;

  private State lastSetpoint = new State();
  private double lastVoltage;

  public SimElevator(ElevatorConfig config) {
    sim =
        new ElevatorSim(
            config.gearbox(),
            config.gearing(),
            config.mass(),
            config.sprocketRadius(),
            config.minHeight(),
            config.maxHeight(),
            true);

    ff = config.ff().createFeedforward();
    pid = config.pid().createPIDController();
  }

  @Override
  public double getHeight() {
    return sim.getPositionMeters();
  }

  @Override
  public State getCurrentState() {
    return new State(getHeight(), sim.getVelocityMetersPerSecond());
  }

  @Override
  public State getDesiredState() {
    return lastSetpoint;
  }

  @Override
  public void updateSetpoint(State setpoint) {
    double ffVoltage = ff.calculate(lastSetpoint.velocity, setpoint.velocity, Constants.PERIOD);
    double pidVoltage = pid.calculate(getHeight(), setpoint.position);

    lastVoltage = ffVoltage + pidVoltage;
    sim.setInputVoltage(lastVoltage);
    sim.update(Constants.PERIOD);

    lastSetpoint = setpoint;
  }

  @Override
  public void stopMoving() {
    lastVoltage = 0;
    sim.setInputVoltage(0);
    sim.update(Constants.PERIOD);
  }

  @Override
  public boolean isFailing() {
    return false;
  }

  @Override
  public double getVoltage() {
    return lastVoltage;
  }

  @Override
  public void close() throws Exception {}
}
