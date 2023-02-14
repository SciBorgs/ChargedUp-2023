package org.sciborgs1155.robot.subsystems;

import static org.sciborgs1155.robot.Constants.Intake.*;
import static org.sciborgs1155.robot.Ports.Intake.*;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;

public class Intake extends SubsystemBase implements Loggable {
  public enum GamePieces {
    NOTHING(new Color(0, 0, 0)),
    CONE(new Color(255, 214, 7)),
    CUBE(new Color(128, 0, 128));

    public final Color color;

    private GamePieces(Color color) {
      this.color = color;
    }
  }

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 sensor = new ColorSensorV3(i2cPort);
  private final ColorMatch matcher = new ColorMatch();

  public Intake() {
    matcher.addColorMatch(GamePieces.CONE.color);
    matcher.addColorMatch(GamePieces.CUBE.color);
  }

  public GamePieces getGamePiece() {
    Color detected = sensor.getColor();
    ColorMatchResult match = matcher.matchClosestColor(detected);

    if (match.color == GamePieces.CONE.color) {
      return GamePieces.CONE;
    } else if (match.color == GamePieces.CUBE.color) {
      return GamePieces.CUBE;
    } else {
      return GamePieces.NOTHING;
    }
  }

  @Override
  public void periodic() {
    GamePieces piece = getGamePiece();
    if (piece != GamePieces.NOTHING) {
      System.out.println(piece.name());
    }
  }

  // @Log(name = "applied output", methodName = "getAppliedOutput")
  // private final CANSparkMax wheels = Motors.INTAKE.build(MotorType.kBrushless, WHEEL_MOTOR);

  // @Log(name = "velocity", methodName = "getVelocity")
  // private final RelativeEncoder encoder = wheels.getEncoder();

  // public Command start() {
  //   return runOnce(() -> wheels.set(WHEEL_SPEED));
  // }

  // public Command stop() {
  //   return runOnce(wheels::stopMotor);
  // }

  // public Command run() {
  //   return startEnd(() -> wheels.set(WHEEL_SPEED), wheels::stopMotor);
  // }
}
