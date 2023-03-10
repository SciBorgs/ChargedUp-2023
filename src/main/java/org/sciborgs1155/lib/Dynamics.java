package org.sciborgs1155.lib;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.system.plant.DCMotor;
import org.sciborgs1155.robot.Constants.Arm.*;
import org.sciborgs1155.robot.Constants.Dimensions;

public class Dynamics {
  private final SimpleMotorFeedforward wristFeedforward =
      new SimpleMotorFeedforward(Wrist.kS, Wrist.kV, Wrist.kA);
  private final SimpleMotorFeedforward elbowFeedforward =
      new SimpleMotorFeedforward(Elbow.kS, Elbow.kV, Elbow.kA);

  private final DCMotor motor = DCMotor.getNEO(1);

  /**
   * @param position The position setpoints for the elbow and wrist
   * @param velocity The velocity setpoints for the elbow and wrist
   * @return Returns a vector of voltages for the elbow and wrist, respectively
   */
  public Vector<N2> calculate(Vector<N2> position, Vector<N2> velocity, Vector<N2> accel) {
    Matrix<N2, N1> tg = Tg(position);
    return VecBuilder.fill(
        elbowFeedforward.calculate(position.get(0, 0), velocity.get(0, 0), accel.get(0, 0))
            + motor.getVoltage(tg.get(0, 0), velocity.get(0, 0)),
        wristFeedforward.calculate(position.get(1, 0), velocity.get(1, 0), accel.get(1, 0))
            + motor.getVoltage(tg.get(1, 0), velocity.get(1, 0)));
  }

  /**
   * @param position A vector containing the position setpoints of the elbow and the wrist,
   *     respectively
   * @return Calculates and returns the torque applied due to gravity
   */
  private Matrix<N2, N1> Tg(Vector<N2> position) {
    Matrix<N2, N1> tg = new Matrix<>(N2.instance, N1.instance);

    // Forearm
    tg.set(
        0,
        0,
        ((Dimensions.FOREARM_MASS * Dimensions.FOREARM_RADIUS
                    + Dimensions.CLAW_MASS * Dimensions.FOREARM_LENGTH)
                * 9.81
                * Math.cos(position.get(0, 0)))
            + Dimensions.CLAW_MASS
                * Dimensions.CLAW_RADIUS
                * 9.81
                * Math.cos(position.get(0, 0) + position.get(0, 1)));

    // Claw
    tg.set(
        1,
        0,
        Dimensions.CLAW_MASS
            * Dimensions.CLAW_RADIUS
            * 9.81
            * Math.cos(position.get(0, 0) + position.get(0, 1)));

    return tg;
  }
}
