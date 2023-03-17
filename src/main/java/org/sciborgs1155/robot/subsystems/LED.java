package org.sciborgs1155.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Constants.led;
import org.sciborgs1155.robot.Ports;
import org.sciborgs1155.robot.commands.Scoring.*;

public class LED extends SubsystemBase {

  private static AddressableLED led1;
  // private static AddressableLED led2;
  private static AddressableLEDBuffer led1Buffer;
  // private static AddressableLEDBuffer led2Buffer;

  private int rainbowFirstPixelHue;

  private LEDColor currColor;

  public enum LEDColor {
    RAINBOW,
    PG,
    BLUE,
    YELLOW
  }

  public LED() {
    led1 = new AddressableLED(Ports.LED.led1);
    led1Buffer = new AddressableLEDBuffer(Constants.led.buffer1Length);
    led1.setLength(led1Buffer.getLength());
    led1.start();
    new AddressableLEDSim(led1);

    // led2 = new AddressableLED(Ports.LED.led2);
    // led2Buffer = new AddressableLEDBuffer(Constants.led.led2Buffer);
    // led2.setLength(Constants.led.led2Length);
    // led2.start();
  }

  public void setColor(LEDColor desiredColor) {
    currColor = desiredColor;

    if (desiredColor == LEDColor.RAINBOW) {
      rainbow();
    }
    if (desiredColor == LEDColor.PG) {
      pgLED();
    }
    if (desiredColor == LEDColor.BLUE) {
      cubeLED();
    }
    if (desiredColor == LEDColor.YELLOW) {
      coneLED();
    }
  }

  public static void coneLED() {
    for (int i = 0; i < led1Buffer.getLength(); i++) {
      led1Buffer.setRGB(i, led.yellow.getRed(), led.yellow.getGreen(), led.yellow.getBlue());
      // led2Buffer.setLED(i, Color.kYellow);
    }
    led1.setData(led1Buffer);
    // led2.setData(led2Buffer);
  }

  public static void cubeLED() {
    for (int i = 0; i < led1Buffer.getLength(); i++) {
      led1Buffer.setRGB(i, led.blue.getRed(), led.blue.getGreen(), led.blue.getBlue());
      // led2Buffer.setLED(i, Color.kBlue);
    }
    led1.setData(led1Buffer);
    // led2.setData(led2Buffer);
  }

  public Command gamepiece(GamePiece gamePiece) {
    if (gamePiece == GamePiece.CONE) {
      return Commands.run(() -> coneLED());
    } else if (gamePiece == GamePiece.CUBE) {
      return Commands.run(() -> cubeLED());
    }
    return sbLED();
  }

  public void rainbow() {
    for (var i = 0; i < led1Buffer.getLength(); i++) {
      final var hue = (rainbowFirstPixelHue + (i * 180 / led1Buffer.getLength())) % 180;

      led1Buffer.setHSV(i, hue, 255, 128);
    }
    rainbowFirstPixelHue += 3;
    rainbowFirstPixelHue %= 180;
  }

  /** Black and Yellow */
  public static Command sbLED() {
    for (int i = 0; i < led1Buffer.getLength(); i++) {
      led1Buffer.setRGB(i, led.yellow.getRed(), led.yellow.getGreen(), led.yellow.getBlue());
    }
    led1.setData(led1Buffer);

    // for (int i = 0; i < led2Buffer.getLength(); i++) {
    //   led2Buffer.setLED(i, Color.kYellow);
    // }
    // led2.setData(led2Buffer);

    return Commands.runOnce(() -> sbLED());
  }

  /** Purple and green */
  public static void pgLED() {
    for (int i = 0; i < led1Buffer.getLength(); i++) {
      led1Buffer.setRGB(
          i, led.lightPurple.getRed(), led.lightPurple.getGreen(), led.lightPurple.getBlue());
    }
    led1.setData(led1Buffer);

    // for (int i = 0; i < led2Buffer.getLength(); i++) {
    //   led2Buffer.setLED(i, Color.kLimeGreen);
    // }
    // led2.setData(led2Buffer);
  }

  @Override
  public void periodic() {
    switch (currColor) {
      case RAINBOW:
        rainbow();

        break;
      case PG:
        pgLED();

        break;
      case YELLOW:
        coneLED();

        break;
      case BLUE:
        cubeLED();

        break;
    }

    led1.setData(led1Buffer);
  }
}
