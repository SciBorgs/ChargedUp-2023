package org.sciborgs1155.lib;

import org.ejml.data.DMatrix3;
import org.ejml.data.DMatrix3x3;

/**
 * Kinematic model for a double-jointed arm on an elevator.
 * 
 * Motor model is from Team 449.
 */
public class ElevatorArmClawKinematics {

    private final double M_arm, M_claw, M_casc;
    private final double I_arm, I_claw, L_arm, L_claw, Lc_arm, Lc_claw;

    private final double K_t, K_v, R;
    private final double G_arm, G_claw, G_casc, N_arm, N_claw, N_casc;

    public ElevatorArmClawKinematics(double m_arm, double m_claw, double m_casc, double i_arm, double i_claw,
            double l_arm, double l_claw, double lc_arm, double lc_claw, double k_t, double k_v, double r, double g_arm,
            double g_claw, double g_casc, double n_arm, double n_claw, double n_casc) {
        M_arm = m_arm;
        M_claw = m_claw;
        M_casc = m_casc;
        I_arm = i_arm;
        I_claw = i_claw;
        L_arm = l_arm;
        L_claw = l_claw;
        Lc_arm = lc_arm;
        Lc_claw = lc_claw;
        K_t = k_t;
        K_v = k_v;
        R = r;
        G_arm = g_arm;
        G_claw = g_claw;
        G_casc = g_casc;
        N_arm = n_arm;
        N_claw = n_claw;
        N_casc = n_casc;
    }

}
