package org.sciborgs1155.lib;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

/**
 * Kinematic model for a double-jointed arm on an elevator.
 *
 * <p>Motor model is from Team 449.
 */
public class PlacementDynamics {

  private static final double g = 9.80665;

  private final ArmConstants arm, claw;
  private final double cascMass;

  private final DMatrixRMaj B, B_inv, K_resistive, M_diag;

  record ArmConstants(double mass, double moi, double length, double radius) {}

  record MotorConstants(double G, double N, double K_f) {}

  public PlacementDynamics(
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
      DMatrixRMaj applied = gravity(state.pos); // equilibrium torques/forces
      CommonOps_DDRM.multAdd(M(state.pos), desiredAcceleration.toVector(), applied);
      CommonOps_DDRM.multAdd(C(state), state.vel.toVector(), applied);
      CommonOps_DDRM.multAdd(K_resistive, state.vel.toVector(), applied);

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

    public State getState() {
      return this.state;
    }

    void update(MotorOutputs inputs, double dt) {
      // Mq.. = Bu - Text - Cq. - Tg
      DMatrixRMaj force = new DMatrixRMaj(3, 1);
      CommonOps_DDRM.multAdd(B, inputs.toVector(), force);
      CommonOps_DDRM.subtractEquals(force, gravity(state.pos));
      CommonOps_DDRM.multAdd(-1, C(state), state.vel.toVector(), force);
      CommonOps_DDRM.multAdd(-1, K_resistive, state.vel.toVector(), force);

      DMatrixRMaj velocity = state.vel.toVector();
      DMatrixRMaj M_inv = new DMatrixRMaj(3, 3);
      CommonOps_DDRM.invertSPD(M(state.pos), M_inv);
      CommonOps_DDRM.multAdd(dt, M_inv, force, velocity);

      state =
          new State(
              new Configuration(
                  state.pos.angleArm + dt * state.vel.angleArm,
                  state.pos.angleClaw + dt * state.vel.angleClaw,
                  state.pos.height + dt * state.vel.height),
              Configuration.fromVector(velocity));
    }
  }

  public Simulation getSimulation() {
    return new Simulation();
  }
}
