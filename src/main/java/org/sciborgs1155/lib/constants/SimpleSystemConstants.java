package org.sciborgs1155.lib.constants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public record SimpleSystemConstants(double ks, double kv, double ka) {

  public SimpleMotorFeedforward feedforward() {
    return new SimpleMotorFeedforward(ks, kv, ka);
  }

  public LinearSystem<N2, N1, N1> positionSystem() {
    return LinearSystemId.identifyPositionSystem(kv, ka);
  }

  public LinearSystem<N1, N1, N1> velocitySystem() {
    return LinearSystemId.identifyVelocitySystem(kv, ka);
  }

  public DCMotorSim sim(DCMotor gearbox, double gearing) {
    var system =
        new LinearSystem<N2, N1, N2>(
            Matrix.mat(Nat.N2(), Nat.N2()).fill(0.0, 1.0, 0.0, -kv / ka),
            Matrix.mat(Nat.N2(), Nat.N1()).fill(0, 1.0 / ka),
            Matrix.eye(Nat.N2()),
            Matrix.mat(Nat.N2(), Nat.N1()).fill(0.0, 0.0));

    return new DCMotorSim(system, gearbox, gearing);
  }

  public static record ArmSystemConstants(SimpleSystemConstants system, double kg) {}

  public static record ElevatorSystemConstants(SimpleSystemConstants system, double kg) {}
}
