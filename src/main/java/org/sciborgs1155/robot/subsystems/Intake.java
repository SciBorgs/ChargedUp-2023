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
  private final CANSparkMax wheels = MOTOR.build(MotorType.kBrushless, WHEEL_MOTOR);

  @Log(name = "velocity", methodName = "getVelocity")
  private final RelativeEncoder encoder = wheels.getEncoder();

  public Command start(boolean reversed) {
    return runOnce(() -> wheels.set(reversed ? -WHEEL_SPEED : WHEEL_SPEED));
  }

  public Command stop() {
    return runOnce(wheels::stopMotor);
  }

  public Command run() {
    return startEnd(() -> wheels.set(WHEEL_SPEED), wheels::stopMotor);
  }

  @Override
  public void close() {
    wheels.close();
  }
}
