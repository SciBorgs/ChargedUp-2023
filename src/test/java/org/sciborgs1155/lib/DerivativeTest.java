package org.sciborgs1155.lib;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.RepeatedTest;

public class DerivativeTest {

  @BeforeEach
  void setup() {
    assert HAL.initialize(500, 0);
  }

  @RepeatedTest(3)
  void derivative() throws InterruptedException {
    Derivative dt = new Derivative();
    dt.calculate(1);
    Thread.sleep(500);
    assertEquals(2, dt.calculate(2), 7e-2);
    Thread.sleep(500);
    assertEquals(-2, dt.calculate(1), 7e-2);
  }
}
