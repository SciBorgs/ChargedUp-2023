package org.sciborgs1155.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import java.util.List;
import org.sciborgs1155.lib.BetterElevatorFeedforward;
import org.sciborgs1155.lib.failure.HardwareFault;
import org.sciborgs1155.robot.Constants;

public class SimElevator implements ElevatorIO {

  private final ElevatorSim sim;

  private final PIDController pid;
  private final BetterElevatorFeedforward ff;

  private State lastSetpoint = new State();
  private double lastVoltage;

  public SimElevator(ElevatorConfig config, double startingHeight) {
    sim =
        new ElevatorSim(
            config.gearbox(),
            config.gearing(),
            config.mass(),
            config.sprocketRadius(),
            config.minHeight(),
            config.maxHeight(),
            true,
            VecBuilder.fill(0.005));

    sim.setState(VecBuilder.fill(startingHeight, 0));

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
    sim.setInputVoltage(MathUtil.clamp(lastVoltage, -12, 12));
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
  public double getVoltage() {
    return lastVoltage;
  }

  @Override
  public List<HardwareFault> getFaults() {
    return List.of();
  }

  @Override
  public void close() throws Exception {}
}
