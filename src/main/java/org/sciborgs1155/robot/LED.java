package org.sciborgs1155.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.sciborgs1155.robot.commands.Scoring.*;

public class LED extends SubsystemBase {
  private static AddressableLED led1;
  private static AddressableLED led2;
  private static AddressableLEDBuffer led1Buffer;
  private static AddressableLEDBuffer led2Buffer;

  public LED() {
    led1 = new AddressableLED(Ports.LED.led1);
    led1Buffer = new AddressableLEDBuffer(Constants.led.led1Buffer);
    led1.setLength(Constants.led.led1Lenght);
    led1.start();

    led2 = new AddressableLED(Ports.LED.led2);
    led2Buffer = new AddressableLEDBuffer(Constants.led.led2Buffer);
    led2.setLength(Constants.led.led2Lenght);
    led2.start();
  }

  public static void coneLED() {
    for (int i = 0; i < led1Buffer.getLength(); i++) {
      led1Buffer.setRGB(i, 226, 248, 31);
    }
    led1.setData(led1Buffer);

    for (int i = 0; i < led2Buffer.getLength(); i++) {
      led2Buffer.setRGB(i, 226, 248, 31);
    }
    led2.setData(led2Buffer);
  }

  public Command setCone(GamePiece gamePiece) {
    if (gamePiece == GamePiece.CONE) {
      return Commands.runOnce(LED::coneLED);
    }
    return Commands.runOnce(() -> rainbow(0));
  }

  public static void cubeLED() {
    for (int i = 0; i < led1Buffer.getLength(); i++) {
      led1Buffer.setRGB(i, 0, 17, 255);
    }
    led1.setData(led1Buffer);
    led2.setData(led2Buffer);
  }

  public Command setCube(GamePiece gamePiece) {
    if (gamePiece == GamePiece.CUBE) {
      return Commands.runOnce(LED::cubeLED);
    }
    return Commands.runOnce(() -> pgLED());
  }

  public static void rainbow(int rainbowFirstPixelHue) {
    for (int i = 0; i < led1Buffer.getLength(); i++) {
      final var hue = (rainbowFirstPixelHue + (i * 180 / led1Buffer.getLength())) % 180;
      led1Buffer.setHSV(i, hue, 255, 128);
    }
    rainbowFirstPixelHue += 3;
    rainbowFirstPixelHue %= 180;
    led1.setData(led1Buffer);
    led2.setData(led2Buffer);
  }

  /** Purple and green */
  public static void pgLED() {
    for (int i = 0; i < led1Buffer.getLength(); i++) {
      led1Buffer.setRGB(i, 151, 115, 210);
    }
    led1.setData(led1Buffer);

    for (int i = 0; i < led2Buffer.getLength(); i++) {
      led2Buffer.setRGB(i, 142, 224, 69);
    }
    led2.setData(led2Buffer);
  }

  @Override
  public void periodic() {
    rainbow(0);
    led1.setData(led1Buffer);
    led2.setData(led2Buffer);
  }
}
