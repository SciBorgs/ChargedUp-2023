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
import org.sciborgs1155.lib.CustomPeriodRunnables;

public class Intake extends SubsystemBase implements Loggable {
  public enum GamePieces {
    NOTHING(new Color(0, 0, 0)),
    CONE(new Color(83, 133, 33)),
    CUBE(new Color(60, 103, 93));

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

    CustomPeriodRunnables.add(this::test, 0.5);
  }

  public GamePieces getGamePiece() {
    Color detected = sensor.getColor();

    double r = detected.red * 255;
    double g = detected.green * 255;
    double b = detected.blue * 255;

    System.out.println(r + " " + g + " " + b);

    if (r >= 70 && r <= 95 && g >= 120 && g <= 145 && b >= 15 && b <= 50) {
      /*
       * old values were:
       * r >= 75 && r <= 92 && g >= 127 && g <= 141 && b >= 20 && b <= 46
       *
       */
      return GamePieces.CONE;
    } else if (r >= 50 && r <= 70 && g >= 80 && g <= 125 && b >= 65 && b <= 120) {
      /*
       * old values were:
       * r >= 55 && r <= 67 && g >= 87 && g <= 120 && b >= 69 && b <= 112
       *
       */
      return GamePieces.CUBE;
    } else {
      return GamePieces.NOTHING;
    }
  }

  public GamePieces getGamePieceByMatching() {
    Color detected = sensor.getColor();
    ColorMatchResult match = matcher.matchClosestColor(detected);

    System.out.println(detected.red * 255 + " " + detected.green * 255 + " " + detected.blue * 255);

    if (match.color == GamePieces.CONE.color) {
      return GamePieces.CONE;
    } else if (match.color == GamePieces.CUBE.color) {
      return GamePieces.CUBE;
    } else {
      return GamePieces.NOTHING;
    }
  }

  public void test() {
    GamePieces piece = getGamePiece();
    System.out.println(sensor.getProximity());
    System.out.println(piece.name());
  }

  public void testMatch() {
    GamePieces piece = getGamePieceByMatching();
    System.out.println(sensor.getProximity());
    System.out.println(piece.name());
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
