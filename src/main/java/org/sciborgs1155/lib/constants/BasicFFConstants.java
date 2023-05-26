package org.sciborgs1155.lib.constants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public record BasicFFConstants(double s, double v, double a) {

  public BasicFFConstants(double s, double v) {
    this(s, v, 0);
  }

  public SimpleMotorFeedforward createFeedforward() {
    return new SimpleMotorFeedforward(s, v, a);
  }

  public DCMotorSim sim(DCMotor gearbox, double gearing) {
    // temporary until DCMotorSim patch gets upstream
    var system =
        new LinearSystem<N2, N1, N2>(
            Matrix.mat(Nat.N2(), Nat.N2()).fill(0.0, 1.0, 0.0, -v / a),
            Matrix.mat(Nat.N2(), Nat.N1()).fill(0, 1.0 / a),
            Matrix.eye(Nat.N2()),
            Matrix.mat(Nat.N2(), Nat.N1()).fill(0.0, 0.0));

    return new DCMotorSim(system, gearbox, gearing);
  }
}
