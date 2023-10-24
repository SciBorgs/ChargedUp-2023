package org.sciborgs1155.robot.subsystems;

import static org.sciborgs1155.robot.Constants.Auto.*;
import static org.sciborgs1155.robot.Constants.Intake.*;
import static org.sciborgs1155.robot.Ports.Intake.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import java.util.List;
import java.util.function.Supplier;
import org.sciborgs1155.lib.SparkUtils;
import org.sciborgs1155.lib.failure.Fallible;
import org.sciborgs1155.lib.failure.FaultBuilder;
import org.sciborgs1155.lib.failure.HardwareFault;
import org.sciborgs1155.robot.subsystems.arm.ArmState.GamePiece;

public class Intake extends SubsystemBase implements Fallible, Loggable, AutoCloseable {

  @Log(name = "applied output", methodName = "getAppliedOutput")
  @Log(name = "current", methodName = "getOutputCurrent")
  private final CANSparkMax wheels =
      SparkUtils.create(WHEEL_MOTOR, MotorType.kBrushless, MOTOR_CFG);

  @Log(name = "velocity", methodName = "getVelocity")
  private final RelativeEncoder encoder = wheels.getEncoder();

  @Log private double coneSpeed = CONE_SPEED;
  @Log private double cubeSpeed = CUBE_SPEED;

  public Command set(double percent) {
    return run(() -> wheels.set(MathUtil.clamp(percent, -1, 1)))
        .finallyDo(end -> wheels.stopMotor());
  }

  public Command intake(Supplier<GamePiece> gamePiece) {
    return set(gamePiece.get() == GamePiece.CONE ? coneSpeed : cubeSpeed).withName("intaking");
  }

  public Command outtake(Supplier<GamePiece> gamePiece) {
    return set(-1 * (gamePiece.get() == GamePiece.CONE ? coneSpeed : cubeSpeed))
        .withName("outtaking");
  }

  public Command outtakeWithTimeout(Supplier<GamePiece> gamePiece) {
    return outtake(gamePiece)
        .withTimeout(
            switch (gamePiece.get()) {
              case CONE -> CONE_OUTTAKE_TIME;
              case CUBE -> CUBE_OUTTAKE_TIME;
            });
  }

  public Command stop() {
    return runOnce(wheels::stopMotor);
  }

  public boolean isHoldingItem() {
    return wheels.getOutputCurrent() > THRESHOLD;
  }

  @Override
  public List<HardwareFault> getFaults() {
    return FaultBuilder.create().register("intake spark", wheels).build();
  }

  @Override
  public void close() {
    wheels.close();
  }
}
