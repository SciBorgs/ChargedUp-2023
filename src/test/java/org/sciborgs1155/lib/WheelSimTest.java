import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;
import org.sciborgs1155.lib.WheelSim;

public class WheelSimTest {
  @Test
  public void thing() {
    WheelSim sim = new WheelSim(1, 1);

    sim.setInput(1);
    sim.update(10);
    System.out.println(sim.getVelocity());
    System.out.println(sim.getPosition());
    assert sim.getVelocity() > 0;
    assert sim.getPosition() > 0;
  }
}
