package org.sciborgs1155.lib;

import org.ejml.data.DMatrix3;
import org.ejml.data.DMatrix3x3;
import org.ejml.dense.fixed.CommonOps_DDF3;

/**
 * Kinematic model for a double-jointed arm on an elevator.
 * 
 * Motor model is from Team 449.
 */
public class PlacementKinematics {

    private static final double g = 9.80665;

    private final ArmConstants arm, claw;
    private final double cascMass;

    private final DMatrix3x3 B, B_inv, K_resistive, M_diag;

    record ArmConstants(double mass, double moi, double length, double radius) {}
    record MotorConstants(double G, double N, double K_f) {}

    public PlacementKinematics(
            ArmConstants arm, 
            ArmConstants claw, 
            double cascMass,
            MotorConstants armMotor, 
            MotorConstants clawMotor, 
            MotorConstants cascMotor,
            double R, double K_t, double K_v) {
        this.arm = arm;
        this.claw = claw;
        this.cascMass = cascMass;

        // Feedthrough
        B = new DMatrix3x3();
        CommonOps_DDF3.mult(
            diag(armMotor.G, clawMotor.G, cascMotor.G), 
            diag(armMotor.N, clawMotor.N, cascMotor.N), B);
        CommonOps_DDF3.scale(K_t / R, B);

        B_inv = new DMatrix3x3();
        CommonOps_DDF3.invert(B, B_inv);

        // backemf + friction
        K_resistive = new DMatrix3x3();
        CommonOps_DDF3.mult(diag(armMotor.G, clawMotor.G, cascMotor.G), B, K_resistive);
        CommonOps_DDF3.scale(1 / K_v, K_resistive);
        CommonOps_DDF3.addEquals(K_resistive, diag(armMotor.K_f, clawMotor.K_f, cascMotor.K_f));

        M_diag = diag(arm.moi + arm.mass * arm.radius * arm.radius + claw.mass * arm.length * arm.length,
                      claw.moi + claw.mass * claw.radius * claw.radius,
                      arm.mass + claw.mass + cascMass);
    }

    public record Configuration(double angleArm, double angleClaw, double height) {
        DMatrix3 toVector() {
            return new DMatrix3(angleArm, angleClaw, height);
        }

        static Configuration fromVector(DMatrix3 vector) {
            return new Configuration(vector.a1, vector.a2, vector.a3);
        }
    }

    public record State(Configuration pos, Configuration vel) {}

    private DMatrix3x3 M(Configuration pos) {
        DMatrix3x3 M = new DMatrix3x3();

        DMatrix3x3 M_LU = new DMatrix3x3(0, 0, 0,
            arm.length * claw.radius * claw.mass * Math.cos(pos.angleClaw - pos.angleArm), 0, 0,
            (arm.length * claw.mass + arm.radius * arm.mass) * Math.cos(pos.angleArm), claw.radius * claw.mass * Math.cos(pos.angleClaw), 0
        );

        CommonOps_DDF3.transpose(M_LU, M);
        CommonOps_DDF3.addEquals(M, M_LU);
        CommonOps_DDF3.addEquals(M, M_diag);

        return M;
    }

    private DMatrix3x3 C(State state) {
        DMatrix3x3 C = new DMatrix3x3();
        C.set(0, 1,  state.vel.angleClaw * arm.length * claw.radius * claw.mass * Math.sin(state.pos.angleArm - state.pos.angleClaw));
        C.set(1, 0,  state.vel.angleArm  * arm.length * claw.radius * claw.mass * Math.sin(state.pos.angleClaw - state.pos.angleArm));
        C.set(2, 0, -state.vel.angleArm  * (arm.length * claw.mass + arm.radius * arm.mass) * Math.sin(state.pos.angleArm));
        C.set(2, 1, -state.vel.angleClaw * claw.radius * claw.mass * Math.sin(state.pos.angleClaw));
        return C;
    }

    private DMatrix3 gravity(Configuration pos) {
        return new DMatrix3(
            g * Math.cos(pos.angleArm) * (arm.length * claw.mass + arm.radius * arm.mass),
            g * claw.radius * claw.mass * Math.cos(pos.angleClaw),
            g * (arm.mass + claw.mass + cascMass)
        );
    }

    // TODO move out to matrix util
    private static DMatrix3x3 diag(double a11, double a22, double a33) {
        return new DMatrix3x3(a11, 0, 0, 0, a22, 0, 0, 0, a33);
    }

    public record MotorOutputs(double voltageElevator, double voltageArm, double voltageClaw) {
        public static MotorOutputs fromVector(DMatrix3 voltages) {
            return new MotorOutputs(voltages.get(0, 0), voltages.get(1, 0), voltages.get(2, 0));
        }

        public DMatrix3 toVector() {
            return new DMatrix3(voltageElevator, voltageArm, voltageClaw);
        }
    }

    class Feedforward {
        private Feedforward() {}

        MotorOutputs calculate(State state, Configuration desiredAcceleration) {
            DMatrix3 inertial = new DMatrix3(), coriolis = new DMatrix3(), resistive = new DMatrix3();
            CommonOps_DDF3.mult(M(state.pos), desiredAcceleration.toVector(), inertial);
            CommonOps_DDF3.mult(C(state), state.vel.toVector(), coriolis);
            CommonOps_DDF3.mult(K_resistive, state.vel.toVector(), resistive);

            DMatrix3 applied = gravity(state.pos); // equilibrium torques/forces
            CommonOps_DDF3.addEquals(applied, inertial);
            CommonOps_DDF3.addEquals(applied, coriolis);
            CommonOps_DDF3.addEquals(applied, resistive);

            DMatrix3 voltages = new DMatrix3();
            CommonOps_DDF3.mult(B_inv, applied, voltages);
            return MotorOutputs.fromVector(voltages);
        }
    }

    public Feedforward getFeedforward() {
        return new Feedforward();
    }

    class Simulation {
        private State state;
        private Simulation() {}

        public void setState(State state) {
            this.state = state;
        }

        public State getState() {
            return this.state;
        }

        void update(MotorOutputs inputs, double dt) {
            DMatrix3 dVelocity = new DMatrix3();
            
            DMatrix3 coriolis = new DMatrix3(), resistive = new DMatrix3();
            CommonOps_DDF3.mult(C(state), state.vel.toVector(), coriolis);
            CommonOps_DDF3.mult(K_resistive, state.vel.toVector(), resistive);

            DMatrix3 inertia = gravity(state.pos); // equilibrium torques/forces
            CommonOps_DDF3.addEquals(inertia, coriolis);
            CommonOps_DDF3.addEquals(inertia, resistive);

            DMatrix3 force = new DMatrix3();
            CommonOps_DDF3.mult(B, inputs.toVector(), force);
            CommonOps_DDF3.subtractEquals(force, inertia);

            DMatrix3x3 M_inv = M(state.pos);
            CommonOps_DDF3.invert(M_inv, M_inv);
            CommonOps_DDF3.mult(M_inv, force, dVelocity);
            CommonOps_DDF3.scale(dt, dVelocity);

            state = new State(
                new Configuration(
                    state.pos.angleArm  + dt * state.vel.angleArm, 
                    state.pos.angleClaw + dt * state.vel.angleClaw,
                    state.pos.height    + dt * state.vel.height), 
                new Configuration(
                    dVelocity.get(0, 0) + state.vel.angleArm,
                    dVelocity.get(0, 0) + state.vel.angleClaw, 
                    dVelocity.get(0, 0) + state.vel.height)
            );
        }
    }

    public Simulation getSimulation() {
        return new Simulation();
    }
}
