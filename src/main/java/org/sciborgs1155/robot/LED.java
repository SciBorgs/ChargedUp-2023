package org.sciborgs1155.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.sciborgs1155.robot.commands.Scoring.*;

public class LED {
  private AddressableLED led;
  private AddressableLEDBuffer ledBuffer;

  public LED(AddressableLED led, AddressableLEDBuffer ledBuffer) {
    this.led = new AddressableLED(0);
    this.ledBuffer = new AddressableLEDBuffer(0);
  }

  public void coneLED() {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setHSV(i, 57, 94, 98);
    }
    led.setData(ledBuffer);
    led.start();
  }

  public Command setCone(GamePiece gamePiece) {
    if (gamePiece == GamePiece.CONE) {
      return Commands.runOnce(this::coneLED);
    }
    return Commands.runOnce(() -> rainbow(0));
  }

  public void cubeLED() {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setHSV(i, 236, 100, 100);
    }
    led.setData(ledBuffer);
    led.start();
  }

  public Command setCube(GamePiece gamePiece) {
    if (gamePiece == GamePiece.CUBE) {
      return Commands.runOnce(this::cubeLED);
    }
    return Commands.runOnce(() -> pgLED());
  }

  public void rainbow(int rainbowFirstPixelHue) {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      final int hue = (rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
      ledBuffer.setHSV(i, hue, 255, 128);
    }
    rainbowFirstPixelHue += 3;
    rainbowFirstPixelHue %= 180;
    led.setData(ledBuffer);
    led.start();
  }
  /** Purple and green */
  public void pgLED() {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      if (i % 10 >= 5) ledBuffer.setRGB(i, 151, 115, 210);
      else ledBuffer.setRGB(i, 142, 224, 69);
    }
    led.setData(ledBuffer);
    led.start();
  }
}
