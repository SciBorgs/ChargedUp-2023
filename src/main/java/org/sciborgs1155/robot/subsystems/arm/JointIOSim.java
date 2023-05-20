// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.sciborgs1155.robot.subsystems.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import org.sciborgs1155.lib.BetterArmFeedforward;
import org.sciborgs1155.lib.constants.PIDConstants;
import org.sciborgs1155.lib.constants.SystemConstants;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.subsystems.Arm.JointIO;
import org.sciborgs1155.robot.subsystems.Arm.JointConfig;

/** A simulated {@link JointIO} using {@link SingleJointedArmSim} */
public class JointIOSim implements JointIO {

  private final SingleJointedArmSim sim;

  private final PIDController pid;
  private final BetterArmFeedforward ff;

  private Rotation2d baseAngle = new Rotation2d();
  private State setpoint = new State();

  private double voltage;

  public JointIOSim(PIDConstants pid, SystemConstants ff, JointConfig config, boolean gravity) {
    sim =
        new SingleJointedArmSim(
            config.gearbox(),
            config.gearing(),
            SingleJointedArmSim.estimateMOI(config.length(), config.mass()),
            config.length(),
            config.minAngle(),
            config.maxAngle(),
            gravity);

    this.pid = pid.create();
    this.ff = ff.createArmFF();
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
  public void updateDesiredState(State state) {
    double ffVoltage = ff.calculate(setpoint.position + baseAngle.getRadians(), setpoint.velocity, state.velocity, Constants.PERIOD);
    double pidVoltage = pid.calculate(getRelativeAngle().getRadians(), state.position);

    voltage = ffVoltage + pidVoltage;
    sim.setInputVoltage(voltage);
    sim.update(Constants.PERIOD);
    
    setpoint = state;
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
  public double getVoltage() {
      return voltage;
  }

  @Override
  public void close() {}

}
