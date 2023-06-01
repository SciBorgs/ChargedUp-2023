package org.sciborgs1155.robot.subsystems.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.sciborgs1155.lib.BetterArmFeedforward;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.subsystems.arm.JointIO.JointConfig;

/** A simulated {@link JointIO} using {@link SingleJointedArmSim} */
public class SimJoint implements JointIO {

  private final SingleJointedArmSim sim;

  private final PIDController pid;
  private final BetterArmFeedforward ff;

  private Rotation2d baseAngle = new Rotation2d();
  private State setpoint = new State();

  private double voltage;

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
            this.setpoint.velocity,
            setpoint.velocity,
            Constants.PERIOD);
    double pidVoltage = pid.calculate(getRelativeAngle().getRadians(), setpoint.position);

    voltage = ffVoltage + pidVoltage;
    sim.setInputVoltage(voltage);
    sim.update(Constants.PERIOD);

    this.setpoint = setpoint;
  }

  @Override
  public void stopMoving() {
    voltage = 0;
    sim.setInputVoltage(0);
    sim.update(Constants.PERIOD);
  }

  @Override
  public State getDesiredState() {
    return setpoint;
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
  public boolean isFailing() {
    return false;
  }

  @Override
  public double getVoltage() {
    return voltage;
  }

  @Override
  public void close() {}
}
