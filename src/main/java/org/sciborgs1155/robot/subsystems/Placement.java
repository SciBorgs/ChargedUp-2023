package org.sciborgs1155.robot.subsystems;

import org.sciborgs1155.lib.Derivative;
import org.sciborgs1155.robot.Constants.Arm.Elbow;
import org.sciborgs1155.robot.Constants.Arm.Wrist;
import org.sciborgs1155.robot.subsystems.placement.Controller;
import org.sciborgs1155.robot.subsystems.placement.Simulation;
import org.sciborgs1155.robot.subsystems.placement.SolverClient;
import org.sciborgs1155.robot.subsystems.placement.State;
import org.sciborgs1155.robot.subsystems.placement.Trajectory;
import org.sciborgs1155.robot.subsystems.placement.Visualizer;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Placement extends SubsystemBase implements Loggable, AutoCloseable {

    private final Controller controller = new Controller();
    private final Simulation sim = new Simulation();
    private final SolverClient solver = new SolverClient();
    private final Trajectory trajectory;
    @Log private final Visualizer position = new Visualizer(null);
    @Log private final Visualizer setpoint = new Visualizer(null);

    public Vector<N3> getPosition() {

    }

    public Vector<N3> getVelocity() {

    }

    public Vector<N3> getSetpoint() {

    }

    public void setSetpoint(Vector<N3> setpoint) {

    }

    public void setSetpoint(State setpoint) {
        setSetpoint(setpoint.toVec());
    }
    
    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();
    }

    @Override
    public void close() throws Exception {
        // TODO Auto-generated method stub
        
    }
}
