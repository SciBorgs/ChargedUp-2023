package org.sciborgs1155.lib;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

/**
 * Kinematic model for a double-jointed arm on an elevator.
 *
 * <p>Motor model is from Team 449.
 */
public class PlacementKinematics {

  private static final double g = 9.80665;

  private final ArmConstants arm, claw;
  private final double cascMass;

  private final DMatrixRMaj B, B_inv, K_resistive, M_diag;

  record ArmConstants(double mass, double moi, double length, double radius) {}

  record MotorConstants(double G, double N, double K_f) {}

  public PlacementKinematics(
      ArmConstants arm,
      ArmConstants claw,
      double cascMass,
      MotorConstants armMotor,
      MotorConstants clawMotor,
      MotorConstants cascMotor,
      double R,
      double K_t,
      double K_v) {
    this.arm = arm;
    this.claw = claw;
    this.cascMass = cascMass;

    // Feedthrough
    B = new DMatrixRMaj(3, 3);
    CommonOps_DDRM.mult(
        CommonOps_DDRM.diag(armMotor.G, clawMotor.G, cascMotor.G),
        CommonOps_DDRM.diag(armMotor.N, clawMotor.N, cascMotor.N),
        B);
    CommonOps_DDRM.scale(K_t / R, B);

    B_inv = B.createLike();
    CommonOps_DDRM.invert(B, B_inv);

    // backemf + friction
    K_resistive = new DMatrixRMaj(3, 3);
    CommonOps_DDRM.mult(CommonOps_DDRM.diag(armMotor.G, clawMotor.G, cascMotor.G), B, K_resistive);
    CommonOps_DDRM.scale(1 / K_v, K_resistive);
    CommonOps_DDRM.addEquals(
        K_resistive, CommonOps_DDRM.diag(armMotor.K_f, clawMotor.K_f, cascMotor.K_f));

    M_diag =
        CommonOps_DDRM.diag(
            arm.moi + arm.mass * arm.radius * arm.radius + claw.mass * arm.length * arm.length,
            claw.moi + claw.mass * claw.radius * claw.radius,
            arm.mass + claw.mass + cascMass);
  }

  public record Configuration(double angleArm, double angleClaw, double height) {
    DMatrixRMaj toVector() {
      return new DMatrixRMaj(new double[] {angleArm, angleClaw, height});
    }

    static Configuration fromVector(DMatrixRMaj vector) {
      return new Configuration(vector.get(0, 0), vector.get(1, 0), vector.get(2, 0));
    }
  }

  public record State(Configuration pos, Configuration vel) {}

  public DMatrixRMaj M(Configuration pos) {
    DMatrixRMaj M = new DMatrixRMaj(3, 3);

    DMatrixRMaj M_LU =
        new DMatrixRMaj(
            3,
            3,
            true,
            0,
            0,
            0,
            arm.length * claw.radius * claw.mass * Math.cos(pos.angleClaw - pos.angleArm),
            0,
            0,
            (arm.length * claw.mass + arm.radius * arm.mass) * Math.cos(pos.angleArm),
            claw.radius * claw.mass * Math.cos(pos.angleClaw),
            0);

    CommonOps_DDRM.transpose(M_LU, M);
    CommonOps_DDRM.addEquals(M, M_LU);
    CommonOps_DDRM.addEquals(M, M_diag);

    return M;
  }

  public DMatrixRMaj C(State state) {
    DMatrixRMaj C = new DMatrixRMaj(3, 3);
    C.set(
        0,
        1,
        state.vel.angleClaw
            * arm.length
            * claw.radius
            * claw.mass
            * Math.sin(state.pos.angleArm - state.pos.angleClaw));
    C.set(
        1,
        0,
        state.vel.angleArm
            * arm.length
            * claw.radius
            * claw.mass
            * Math.sin(state.pos.angleClaw - state.pos.angleArm));
    C.set(
        2,
        0,
        -state.vel.angleArm
            * (arm.length * claw.mass + arm.radius * arm.mass)
            * Math.sin(state.pos.angleArm));
    C.set(2, 1, -state.vel.angleClaw * claw.radius * claw.mass * Math.sin(state.pos.angleClaw));
    return C;
  }

  private DMatrixRMaj gravity(Configuration pos) {
    return new DMatrixRMaj(
        3,
        1,
        true,
        g * Math.cos(pos.angleArm) * (arm.length * claw.mass + arm.radius * arm.mass),
        g * claw.radius * claw.mass * Math.cos(pos.angleClaw),
        g * (arm.mass + claw.mass + cascMass));
  }

  public record MotorOutputs(double voltageElevator, double voltageArm, double voltageClaw) {
    public static MotorOutputs fromVector(DMatrixRMaj voltages) {
      return new MotorOutputs(voltages.get(0, 0), voltages.get(1, 0), voltages.get(2, 0));
    }

    public DMatrixRMaj toVector() {
      return new DMatrixRMaj(new double[] {voltageElevator, voltageArm, voltageClaw});
    }
  }

  class Feedforward {
    private Feedforward() {}

    public MotorOutputs calculate(State state, Configuration desiredAcceleration) {
      DMatrixRMaj inertial = new DMatrixRMaj(3, 1),
          coriolis = new DMatrixRMaj(3, 1),
          resistive = new DMatrixRMaj(3, 1);
      CommonOps_DDRM.mult(M(state.pos), desiredAcceleration.toVector(), inertial);
      CommonOps_DDRM.mult(C(state), state.vel.toVector(), coriolis);
      CommonOps_DDRM.mult(K_resistive, state.vel.toVector(), resistive);

      DMatrixRMaj applied = gravity(state.pos); // equilibrium torques/forces
      CommonOps_DDRM.addEquals(applied, inertial);
      CommonOps_DDRM.addEquals(applied, coriolis);
      CommonOps_DDRM.addEquals(applied, resistive);

      DMatrixRMaj voltages = new DMatrixRMaj(3, 1);
      CommonOps_DDRM.mult(B_inv, applied, voltages);
      return MotorOutputs.fromVector(voltages);
    }
  }

  public Feedforward getFeedforward() {
    return new Feedforward();
  }

  class Simulation {
    private State state = new State(new Configuration(0, 0, 0), new Configuration(0, 0, 0));

    private Simulation() {}

    public void setState(State state) {
      this.state = state;
    }

    void update(MotorOutputs inputs, double dt) {
      DMatrixRMaj dVelocity = new DMatrixRMaj(3, 1);

      DMatrixRMaj coriolis = new DMatrixRMaj(3, 1), resistive = new DMatrixRMaj(3, 1);
      CommonOps_DDRM.mult(C(state), state.vel.toVector(), coriolis);
      CommonOps_DDRM.mult(K_resistive, state.vel.toVector(), resistive);

      DMatrixRMaj inertia = gravity(state.pos); // equilibrium torques/forces
      CommonOps_DDRM.addEquals(inertia, coriolis);
      CommonOps_DDRM.addEquals(inertia, resistive);

      DMatrixRMaj force = new DMatrixRMaj(3, 1);
      CommonOps_DDRM.mult(B, inputs.toVector(), force);
      CommonOps_DDRM.subtractEquals(force, inertia);

      DMatrixRMaj M_inv = M(state.pos);
      CommonOps_DDRM.invert(M_inv, M_inv);
      CommonOps_DDRM.mult(M_inv, force, dVelocity);
      CommonOps_DDRM.scale(dt, dVelocity);

      state =
          new State(
              new Configuration(
                  state.pos.angleArm + dt * state.vel.angleArm,
                  state.pos.angleClaw + dt * state.vel.angleClaw,
                  state.pos.height + dt * state.vel.height),
              new Configuration(
                  dVelocity.get(0, 0) + state.vel.angleArm,
                  dVelocity.get(1, 0) + state.vel.angleClaw,
                  dVelocity.get(2, 0) + state.vel.height));
    }

    public State getState() {
      return this.state;
    }
  }

  public Simulation getSimulation() {
    return new Simulation();
  }
}
