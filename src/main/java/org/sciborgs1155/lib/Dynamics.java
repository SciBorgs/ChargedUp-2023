package org.sciborgs1155.lib;

import org.sciborgs1155.robot.Constants.Dimensions;
import org.sciborgs1155.robot.Constants.ArmConstants.*;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.system.plant.DCMotor;

public class Dynamics {
    private final SimpleMotorFeedforward wristFeedforward = new SimpleMotorFeedforward(WristConstants.kS, WristConstants.kV, WristConstants.kA);
    private final SimpleMotorFeedforward elbowFeedforward = new SimpleMotorFeedforward(ElbowConstants.kS, ElbowConstants.kV, ElbowConstants.kA);

    // public Vector<N2> calculate(Vector<N2> position, Vector<N2> velocity) {
        
    // }

    private Matrix<N1, N2> Tg(Vector<N2> position) {
        Matrix<N2, N1> tg = new Matrix<>(N2.instance, N1.instance);
        // tg.set(0, 0, )
    }

}
