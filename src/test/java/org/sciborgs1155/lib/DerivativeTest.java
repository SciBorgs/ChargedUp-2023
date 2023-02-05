package org.sciborgs1155.lib;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class DerivativeTest {

  @BeforeEach
  void setup() {
    assert HAL.initialize(500, 0);
  }

  @Test
  void derivative() {
    Derivative dt = new Derivative();
    assertTrue(dt.calculate(2) > 0);
    assertTrue(dt.calculate(1) < 0);
  }
}
