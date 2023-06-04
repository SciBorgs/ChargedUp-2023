package org.sciborgs1155.lib.failure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.List;

@FunctionalInterface
public interface Fallible {
  public List<HardwareFault> getFaults();

  public default boolean isFailing() {
    for (HardwareFault fault : getFaults()) {
      if (!fault.isWarning()) {
        System.out.println("fault: " + fault.description());
        return true;
      }
    }
    return false;
  }

  public default Trigger getTrigger() {
    return new Trigger(this::isFailing);
  }

  public default void onFailing(Command command) {
    getTrigger().debounce(0.5).onTrue(command);
  }
}
