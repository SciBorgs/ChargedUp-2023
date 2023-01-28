package org.sciborgs1155.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.sciborgs1155.robot.Ports.LockingMechanismPorts;

public class LockingMechanism extends SubsystemBase {
  DoubleSolenoid piston =
      new DoubleSolenoid(
          PneumaticsModuleType.CTREPCM,
          LockingMechanismPorts.FORWARD_CHANNEL,
          LockingMechanismPorts.REVERSE_CHANNEL);

  public LockingMechanism() {}

  public void toggleArm() {
    piston.toggle();
  }

  public void extendArm() {
    piston.set(Value.kForward);
  }

  public void retractArm() {
    piston.set(Value.kReverse);
  }

  public Command start() {
    return this.runOnce(() -> extendArm());
  }

  public Command end() {
    return this.runOnce(() -> retractArm());
  }
}
