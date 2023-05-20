package org.sciborgs1155.robot.subsystems.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import org.sciborgs1155.lib.BetterElevatorFeedforward;
import org.sciborgs1155.lib.constants.PIDConstants;
import org.sciborgs1155.lib.constants.SystemConstants;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.subsystems.Arm.ElevatorConfig;
import org.sciborgs1155.robot.subsystems.Arm.ElevatorIO;

public class ElevatorIOSim implements ElevatorIO {

  private final ElevatorSim sim;

  private final PIDController pid;
  private final BetterElevatorFeedforward ff;

  private State setpoint = new State();
  private double voltage;

  public ElevatorIOSim(
      PIDConstants pid, SystemConstants ff, ElevatorConfig config, boolean gravity) {
    sim =
        new ElevatorSim(
            config.gearbox(),
            config.gearing(),
            config.mass(),
            config.sprocketRadius(),
            config.minHeight(),
            config.maxHeight(),
            gravity);
    this.pid = pid.create();
    this.ff = ff.createElevatorFF();
  }

  @Override
  public double getHeight() {
    return sim.getPositionMeters();
  }

  @Override
  public State getState() {
    return new State(getHeight(), sim.getVelocityMetersPerSecond());
  }

  @Override
  public State getDesiredState() {
    return setpoint;
  }

  @Override
  public void update(State setpoint) {
    double ffVoltage = ff.calculate(this.setpoint.velocity, setpoint.velocity, Constants.PERIOD);
    double pidVoltage = pid.calculate(getHeight(), setpoint.position);

    voltage = ffVoltage + pidVoltage;
    sim.setInputVoltage(voltage);
    sim.update(Constants.PERIOD);

    this.setpoint = setpoint;
  }

  @Override
  public boolean isFailing() {
    return false;
  }

  @Override
  public double getVoltage() {
    return voltage;
  }

  @Override
  public void close() throws Exception {}
}
