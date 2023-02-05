package org.sciborgs1155.lib;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class DerivativeTest {

  @BeforeEach
  void setup() {
    assert HAL.initialize(500, 0);
  }

  @Test
  void derivative() throws InterruptedException {
    Derivative dt = new Derivative();
    dt.calculate(1);
    assert roughlyEquals(dt.calculate(1), 0.0);
    Thread.sleep(1000);
    assert roughlyEquals(dt.calculate(2), 1);
  }

  static boolean roughlyEquals(double val1, double val2) {
    return Math.abs(val1 - val2) < 0.3;
  }
}
