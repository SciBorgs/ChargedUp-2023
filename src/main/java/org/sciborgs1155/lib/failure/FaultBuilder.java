package org.sciborgs1155.lib.failure;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.FaultID;
import com.revrobotics.REVLibError;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import java.util.LinkedList;
import java.util.List;

public class FaultBuilder {
  private final List<HardwareFault> faults = new LinkedList<>();

  public List<HardwareFault> faults() {
    return faults;
  }

  public void add(String label, boolean condition) {
    if (condition) {
      faults.add(new HardwareFault(label));
    }
  }

  public void add(HardwareFault fault) {
    faults.add(fault);
  }

  public void add(List<HardwareFault> fault) {
    faults.addAll(fault);
  }

  public void add(String label, CANSparkMax sparkMax) {
    REVLibError err = sparkMax.getLastError();
    if (err != REVLibError.kOk) {
      faults.add(new HardwareFault(String.format("[%s]: Error: %s", label, err.name())));
    }
    for (FaultID id : FaultID.values()) {
      if (sparkMax.getFault(id)) {
        faults.add(new HardwareFault(String.format("[%s]: Fault: %s", label, id.name()), true));
      }
    }
  }

  public void add(String label, DutyCycleEncoder encoder) {
    add(label, !encoder.isConnected());
  }
}
