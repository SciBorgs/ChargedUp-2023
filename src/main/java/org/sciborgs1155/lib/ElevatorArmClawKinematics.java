package org.sciborgs1155.lib;

import org.ejml.data.DMatrix3;
import org.ejml.data.DMatrix3x3;
import org.ejml.dense.fixed.CommonOps_DDF3;

/**
 * Kinematic model for a double-jointed arm on an elevator.
 * 
 * Motor model is from Team 449.
 */
public class ElevatorArmClawKinematics {

    private static final double g = 9.80665;

    private final double M_arm, M_claw, M_casc;
    private final double I_arm, I_claw, L_arm, R_arm, R_claw;

    private final DMatrix3x3 B, B_inv, K_b, M_diag;

    public ElevatorArmClawKinematics(double m_arm, double m_claw, double m_casc, double i_arm, double i_claw,
            double l_arm, double r_arm, double r_claw, double K_t, double K_v, double R, double G_arm,
            double G_claw, double G_casc, int N_arm, int N_claw, int N_casc) {
        M_arm = m_arm;
        M_claw = m_claw;
        M_casc = m_casc;
        I_arm = i_arm;
        I_claw = i_claw;
        L_arm = l_arm;
        R_arm = r_arm;
        R_claw = r_claw;

        B = new DMatrix3x3();
        CommonOps_DDF3.mult(diag(G_arm, G_claw, G_casc), diag(N_arm, N_claw, N_casc), B);
        CommonOps_DDF3.scale(K_t / R, B);

        B_inv = new DMatrix3x3();
        CommonOps_DDF3.invert(B, B_inv);

        K_b = new DMatrix3x3();
        CommonOps_DDF3.mult(diag(G_arm, G_claw, G_casc), B, K_b);
        CommonOps_DDF3.scale(1 / K_v, K_b);

        M_diag = diag(I_arm + M_arm*R_arm*R_arm + M_claw*L_arm*L_arm,
                      I_claw + M_claw*R_claw*R_claw,
                      M_arm + M_claw + M_casc);
    }

    public record State(double angleArm, double angleClaw, double height, double angleArmDot, double angleClawDot, double heightDot) {
        DMatrix3 configurationVector() {
            return new DMatrix3(angleArm, angleClaw, height);
        }

        DMatrix3 velocityVector() {
            return new DMatrix3(angleArmDot, angleClawDot, heightDot);
        }
    };

    private DMatrix3x3 M(State state) {
        DMatrix3x3 M = new DMatrix3x3();

        DMatrix3x3 M_LU = new DMatrix3x3(0, 0, 0,
            L_arm*R_claw*M_claw * Math.cos(state.angleClaw - state.angleArm), 0, 0,
            (L_arm*M_claw + R_arm*M_arm) * Math.cos(state.angleArm), R_claw*M_claw*Math.cos(state.angleClaw), 0
        );

        CommonOps_DDF3.transpose(M_LU, M);
        CommonOps_DDF3.add(M, M_LU, M);
        CommonOps_DDF3.add(M, M_diag, M);

        return M;
    }

    private DMatrix3x3 C(State state) {
        DMatrix3x3 C = new DMatrix3x3();
        C.set(0, 1,  state.angleClawDot * L_arm * R_claw * M_claw * Math.sin(state.angleArm - state.angleClaw));
        C.set(1, 0,  state.angleArmDot  * L_arm * R_claw * M_claw * Math.sin(state.angleClaw - state.angleArm));
        C.set(2, 0, -state.angleArmDot  * (L_arm * M_claw + R_arm * M_arm) * Math.sin(state.angleArm));
        C.set(2, 1, -state.angleClawDot * R_claw * M_claw * Math.sin(state.angleClaw));
        return C;
    }

    private DMatrix3 gravity(State state) {
        return new DMatrix3(
            g * Math.cos(state.angleArm) * (L_arm * M_claw + R_arm * M_arm),
            g * R_claw * M_claw * Math.cos(state.angleClaw),
            g * (M_arm + M_claw + M_casc)
        );
    }

    // TODO move out to matrix util
    private static DMatrix3x3 diag(double a11, double a22, double a33) {
        return new DMatrix3x3(a11, 0, 0, 0, a22, 0, 0, 0, a33);
    }

    public record TargetAcceleration(double armAccel, double clawAccel, double cascadeAccel) {
        DMatrix3 accelerationVector() {
            return new DMatrix3(armAccel, clawAccel, cascadeAccel);
        }
    };

    public record MotorOutputs(double voltageElevator, double voltageArm, double voltageClaw) {
        public static MotorOutputs fromVector(DMatrix3 voltages) {
            return new MotorOutputs(voltages.get(0, 0), voltages.get(1, 0), voltages.get(2, 0));
        }
    }

    class Feedforward {
        private Feedforward() {}

        MotorOutputs calculate(State state, TargetAcceleration desiredAcceleration) {
            DMatrix3 inertial = new DMatrix3(), coriolis = new DMatrix3(), backemf = new DMatrix3();
            CommonOps_DDF3.mult(M(state), desiredAcceleration.accelerationVector(), inertial);
            CommonOps_DDF3.mult(C(state), state.velocityVector(), coriolis);
            CommonOps_DDF3.mult(K_b, state.velocityVector(), backemf);

            DMatrix3 applied = gravity(state); // equilibrium torques/forces
            CommonOps_DDF3.add(applied, inertial, applied);
            CommonOps_DDF3.add(applied, coriolis, applied);
            CommonOps_DDF3.add(applied, backemf , applied);

            DMatrix3 voltages = new DMatrix3();
            CommonOps_DDF3.mult(B_inv, applied, voltages);
            return MotorOutputs.fromVector(voltages);
        }
    }

    public Feedforward getFeedforward() {
        return new Feedforward();
    }

}
