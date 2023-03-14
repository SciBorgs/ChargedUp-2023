package org.sciborgs1155.robot.util;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N6;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.sciborgs1155.robot.Constants.Arm.Elbow;
import org.sciborgs1155.robot.Constants.Dimensions;
import org.sciborgs1155.robot.Constants.Elevator;

public class PlacementSim {

  private final ElevatorSim elevator =
      new ElevatorSim(
          DCMotor.getNEO(3),
          Elevator.CONVERSION.gearing(),
          Dimensions.ELEVATOR_MASS + Dimensions.FOREARM_MASS + Dimensions.CLAW_MASS,
          Elevator.CONVERSION.units(),
          Dimensions.ELEVATOR_MIN_HEIGHT,
          Dimensions.ELEVATOR_MAX_HEIGHT,
          true);

  private final SingleJointedArmSim elbow =
      new SingleJointedArmSim(
          DCMotor.getNEO(3),
          1 / Elbow.CONVERSION.gearing(),
          SingleJointedArmSim.estimateMOI(Dimensions.FOREARM_LENGTH, Dimensions.FOREARM_MASS),
          Dimensions.FOREARM_LENGTH,
          Dimensions.ELBOW_MIN_ANGLE,
          Dimensions.ELBOW_MAX_ANGLE,
          true);

  private final SingleJointedArmSim wrist =
      new SingleJointedArmSim(
          DCMotor.getNEO(1),
          1,
          SingleJointedArmSim.estimateMOI(Dimensions.CLAW_LENGTH, Dimensions.CLAW_MASS),
          Dimensions.CLAW_LENGTH,
          Dimensions.WRIST_MIN_ANGLE,
          Dimensions.WRIST_MAX_ANGLE,
          false);

  public Vector<N6> update(Vector<N3> voltage, double dt) {
    elevator.setInputVoltage(voltage.get(0, 0));
    elbow.setInputVoltage(voltage.get(1, 0));
    wrist.setInputVoltage(voltage.get(2, 0));

    elevator.update(dt);
    elbow.update(dt);
    wrist.update(dt);

    return VecBuilder.fill(
        elevator.getPositionMeters(),
        elbow.getAngleRads(),
        wrist.getAngleRads(),
        elevator.getVelocityMetersPerSecond(),
        elbow.getVelocityRadPerSec(),
        wrist.getVelocityRadPerSec());
  }
}
