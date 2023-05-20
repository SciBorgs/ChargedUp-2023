// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.sciborgs1155.robot.subsystems.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.sciborgs1155.lib.constants.PIDConstants;
import org.sciborgs1155.lib.constants.SystemConstants;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.subsystems.Arm.Joint;
import org.sciborgs1155.robot.subsystems.Arm.JointConfig;

/** A simulated {@link Joint} using {@link SingleJointedArmSim} */
public class JointSim implements Joint {

  private final SingleJointedArmSim sim;

  private final PIDController pid;
  private final ArmFeedforward ff;

  private State setpoint = new State();

  public JointSim(PIDConstants pid, SystemConstants ff, JointConfig config, boolean gravity) {
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
  public Rotation2d getRotation() {
    return Rotation2d.fromRadians(sim.getAngleRads());
  }

  @Override
  public State getState() {
    return new State(sim.getAngleRads(), sim.getVelocityRadPerSec());
  }

  @Override
  public void updateDesiredState(State state) {
    double ffVoltage = ff.calculate(state.position, state.velocity);
    double pidVoltage = pid.calculate(getRotation().getRadians(), state.position);
    sim.setInputVoltage(ffVoltage + pidVoltage);
    sim.update(Constants.PERIOD);
  }

  @Override
  public State getDesiredState() {
    return setpoint;
  }

  @Override
  public void close() {}
}
