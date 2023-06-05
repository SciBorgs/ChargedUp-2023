package org.sciborgs1155.lib.failure;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.FaultID;
import com.revrobotics.REVLibError;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import java.util.LinkedList;
import java.util.List;

public class FaultBuilder {
  private final List<HardwareFault> faults = new LinkedList<>();

  public static FaultBuilder create() {
    return new FaultBuilder();
  }

  public List<HardwareFault> build() {
    return faults;
  }

  public FaultBuilder register(String label, boolean condition) {
    if (condition) {
      faults.add(new HardwareFault(label));
    }
    return this;
  }

  public FaultBuilder register(HardwareFault fault) {
    faults.add(fault);
    return this;
  }

  public FaultBuilder register(List<HardwareFault> fault) {
    faults.addAll(fault);
    return this;
  }

  public FaultBuilder register(String label, CANSparkMax sparkMax) {
    REVLibError err = sparkMax.getLastError();
    if (err != REVLibError.kOk) {
      faults.add(new HardwareFault(String.format("[%s]: Error: %s", label, err.name())));
    }
    for (FaultID id : FaultID.values()) {
      if (sparkMax.getFault(id)) {
        faults.add(new HardwareFault(String.format("[%s]: Fault: %s", label, id.name()), true));
      }
    }
    return this;
  }

  public FaultBuilder register(String label, DutyCycleEncoder encoder) {
    return register(label, !encoder.isConnected());
  }
}
