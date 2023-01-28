package org.sciborgs1155.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticsSubsystem extends SubsystemBase {
  private Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

  public PneumaticsSubsystem() {
    stop();
  }

  public void start() {
    this.compressor.enableDigital();
  }

  public void stop() {
    this.compressor.disable();
  }

  public boolean getStatus() {
    return this.compressor.isEnabled();
  }
}
