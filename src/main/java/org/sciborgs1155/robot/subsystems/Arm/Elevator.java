package org.sciborgs1155.robot.subsystems.Arm;

import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Ports;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Encoder;

public class Elevator implements ElevatorIO{

private State setpoint;

private boolean isfailing;

private final Encoder encoder = new Encoder(Ports.Elevator.ENCODER[0], Ports.Elevator.ENCODER[1], true);
private final double offset = Constants.Elevator.ZERO_OFFSET;

private CANSparkMax lead = Constants.Elevator.MOTOR.build(MotorType.kBrushless, Ports.Elevator.RIGHT_MOTOR);

private CANSparkMax left = Constants.Elevator.MOTOR.build(MotorType.kBrushless, Ports.Elevator.LEFT_MOTOR);
private CANSparkMax right = Constants.Elevator.MOTOR.build(MotorType.kBrushless, Ports.Elevator.MIDDLE_MOTOR);
  
private final PIDController pid = new PIDController(Constants.Elevator.PID.p(), Constants.Elevator.PID.i(), Constants.Elevator.PID.d());
private final ElevatorFeedforward ff = new ElevatorFeedforward(Constants.Elevator.FF.s(), Constants.Elevator.FF.g(), Constants.Elevator.FF.v(), Constants.Elevator.FF.a());

public Elevator(boolean isfailing){
    this.isfailing = isfailing; 

    left.follow(lead);
    right.follow(lead);
    encoder.setDistancePerPulse(Constants.Elevator.RELATIVE_CONVERSION.factor());

    lead.burnFlash();
    left.burnFlash();
    right.burnFlash();

    setpoint = new State();
    this.isfailing = isfailing;
}

    @Override
    public State getDesiredstate() {
        return setpoint;
    }

    @Override
    public double getCurrentposition() {
       return encoder.getDistance() + offset;
    }

    @Override
    public void updateDesiredstate(double height) {
        setpoint = new State(height, setpoint.velocity);
        
        double fbOutput = pid.calculate(getCurrentposition(), setpoint.position);
        double elevatorFF = ff.calculate(setpoint.velocity);
        lead.setVoltage(isFailing() ? 0 : elevatorFF + fbOutput);
    }

    @Override
    public boolean isFailing() {
        return isfailing;
    }

    @Override
    public void close() throws Exception {
        // TODO Auto-generated method stub
    }
}