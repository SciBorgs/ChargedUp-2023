package org.sciborgs1155.lib;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.sciborgs1155.robot.Constants.Dimensions;

import edu.wpi.first.math.system.plant.DCMotor;
import java.util.Random;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.MatrixFeatures_DDRM;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;
import org.sciborgs1155.lib.PlacementDynamics.*;
import org.sciborgs1155.robot.Constants.Arm.ElbowConstants;
import org.sciborgs1155.robot.Constants.Arm.WristConstants;
import org.sciborgs1155.robot.Constants.Elevator;

public class PlacementDynamicsTest {

  public static final DCMotor base = DCMotor.getNeo550(1);
  public static final PlacementDynamics model =
      new PlacementDynamics(
          new ArmConstants(
              Dimensions.FOREARM_MASS,
              Dimensions.FOREARM_MOI,
              Dimensions.FOREARM_LENGTH,
              Dimensions.FOREARM_RADIUS),
          new ArmConstants(
              Dimensions.CLAW_MASS,
              Dimensions.CLAW_MOI,
              Dimensions.CLAW_LENGTH,
              Dimensions.CLAW_RADIUS),
          Dimensions.ELEVATOR_MASS,
          new MotorConstants(Elevator.GEAR_RATIO, 3, 0.1),
          new MotorConstants(ElbowConstants.GEAR_RATIO, 2, 0.1),
          new MotorConstants(WristConstants.GEAR_RATIO, 1, 0.1),
          12 / base.stallCurrentAmps,
          base.stallTorqueNewtonMeters / base.stallCurrentAmps,
          base.freeSpeedRadPerSec / 12);

  public static final Random random = new Random(1155);

  @Test
  void inertiaMatrixIsPositiveDefinite() {
    for (int i = 0; i < 100; i++) {
      DMatrixRMaj inertiaMatrix =
          model.M(
              new Configuration(
                  random.nextDouble(-Math.PI, Math.PI), random.nextDouble(0, Math.PI), 0));
      assertTrue(MatrixFeatures_DDRM.isPositiveDefinite(inertiaMatrix));
    }
  }

  @ParameterizedTest
  @ValueSource(doubles = {0, 0.2, 0.4, 0.6, 0.8, 1.0, 2.0, 4.0})
  public void feedforwardAtEquilibrium(double initialArm) {
    Simulation sim = model.getSimulation();
    sim.setState(new State(new Configuration(initialArm, 0, 5), new Configuration(0, 0, 0)));
    Feedforward ff = model.getFeedforward();
    for (int i = 0; i < 100; i++) {
      sim.update(ff.calculate(sim.getState(), new Configuration(0, 0, 0)), 0.1);
    }
    DMatrixRMaj finalVelocities = sim.getState().vel().toVector();
    for (int i = 0; i < 3; i++) {
      assertTrue(Math.abs(finalVelocities.get(i)) < 1E-6);
    }
  }
}
