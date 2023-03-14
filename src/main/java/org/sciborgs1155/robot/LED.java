package org.sciborgs1155.robot;

import org.sciborgs1155.robot.commands.Scoring.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class LED {
    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;
    private int rainbowFirstPixelHue; 

    public void coneLED() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
          ledBuffer.setHSV(i, 57, 94, 98);
        }
        led.setData(ledBuffer);
    }

    public Command setCone() {
        return Commands.runOnce(this::coneLED);
    }
    
    public void cubeLED() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
          ledBuffer.setHSV(i, 236, 100, 100);
        }
        led.setData(ledBuffer);
    }

    public void rainbow() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
          final int hue = (rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
          // Set the value
          ledBuffer.setHSV(i, hue, 255, 128);
        }
        rainbowFirstPixelHue += 3;
        rainbowFirstPixelHue %= 180;
    }
}
