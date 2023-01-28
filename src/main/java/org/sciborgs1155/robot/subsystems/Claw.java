package org.sciborgs1155.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.sciborgs1155.robot.Constants.Motors;
import org.sciborgs1155.robot.Ports.ClawPorts;
import org.sciborgs1155.robot.Constants;


public class Claw extends SubsystemBase { 
    private CANSparkMax wheels = Motors.INTAKE.buildCanSparkMax(MotorType.kBrushless,ClawPorts.CLAW_WHEELS );
    private CANSparkMax wrist = Motors.INTAKE.buildCanSparkMax(MotorType.kBrushless,ClawPorts.CLAW_WRIST);
    private final PIDController ClawPID = new PIDController(Constants.Claw.kp, Constants.Claw.ki, Constants.Claw.kd);
    private RelativeEncoder wristEncoder;
   
    public Claw(){
        // Enter Encoder Type
        wristEncoder = wrist.getEncoder();
    }
    
    public void turnOnWheels(){
        wheels.set(Constants.Claw.ClawWheelsEnableSpeed);
    }

    public void stopWheels(){
        wheels.set(0);
    }

    public void DisableWrist(){
        wrist.set(0);
    }
    
    public Command runWheels(){
        return this.startEnd(
            () -> this.turnOnWheels(), 
            () -> this.stopWheels()
        );
    }

    
    @Override
    public void periodic(){
        //Check Encoder Output
        wrist.set(ClawPID.calculate(wristEncoder.getPosition()));
    }
}
