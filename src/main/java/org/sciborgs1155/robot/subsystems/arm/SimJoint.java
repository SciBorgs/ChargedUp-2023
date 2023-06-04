package org.sciborgs1155.robot.subsystems.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import java.util.List;
import org.sciborgs1155.lib.BetterArmFeedforward;
import org.sciborgs1155.lib.failure.HardwareFault;
import org.sciborgs1155.robot.Constants;

/** A simulated {@link JointIO} using {@link SingleJointedArmSim} */
public class SimJoint implements JointIO {

  private final SingleJointedArmSim sim;

  private final PIDController pid;
  private final BetterArmFeedforward ff;

  private Rotation2d baseAngle = new Rotation2d();
  private State lastSetpoint = new State();
  private double lastVoltage;

  public SimJoint(JointConfig config, boolean gravity) {
    sim =
        new SingleJointedArmSim(
            config.gearbox(),
            config.gearing(),
            SingleJointedArmSim.estimateMOI(config.length(), config.mass()),
            config.length(),
            config.minAngle(),
            config.maxAngle(),
            gravity);

    ff = config.ff().createFeedforward();
    pid = config.pid().createPIDController();
  }

  @Override
  public Rotation2d getRelativeAngle() {
    return Rotation2d.fromRadians(sim.getAngleRads());
  }

  @Override
  public State getCurrentState() {
    return new State(sim.getAngleRads(), sim.getVelocityRadPerSec());
  }

  @Override
  public void updateSetpoint(State setpoint) {
    double ffVoltage =
        ff.calculate(
            setpoint.position + baseAngle.getRadians(),
            lastSetpoint.velocity,
            setpoint.velocity,
            Constants.PERIOD);
    double pidVoltage = pid.calculate(getRelativeAngle().getRadians(), setpoint.position);

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
  public State getDesiredState() {
    return lastSetpoint;
  }

  @Override
  public void setBaseAngle(Rotation2d baseAngle) {
    this.baseAngle = baseAngle;
  }

  @Override
  public Rotation2d getBaseAngle() {
    return baseAngle;
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
  public void close() {}
}
