package org.sciborgs1155.robot.subsystems;

import static org.sciborgs1155.robot.Constants.Intake.*;
import static org.sciborgs1155.robot.Ports.Intake.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Intake extends SubsystemBase implements Loggable, AutoCloseable {

  @Log(name = "applied output", methodName = "getAppliedOutput")
  @Log(name = "current", methodName = "getOutputCurrent")
  private final CANSparkMax wheels = MOTOR.build(MotorType.kBrushless, WHEEL_MOTOR);

  @Log(name = "velocity", methodName = "getVelocity")
  private final RelativeEncoder encoder = wheels.getEncoder();

  @Log private double intakeSpeed = INTAKE_SPEED;
  @Log private double outtakeSpeed = OUTTAKE_SPEED;

  public Command intake() {
    return run(() -> wheels.set(intakeSpeed)).withName("intake");
  }

  public Command outtake() {
    return run(() -> wheels.set(outtakeSpeed)).withName("outtake");
  }

  public Command stop() {
    return runOnce(wheels::stopMotor);
  }

  public boolean isHoldingItem() {
    return wheels.getOutputCurrent() > THRESHOLD;
  }

  @Override
  public void close() {
    wheels.close();
  }
}
