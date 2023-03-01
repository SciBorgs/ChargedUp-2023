package org.sciborgs1155.lib;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

public class DerivativeTest {

  @BeforeEach
  void setup() {
    assert HAL.initialize(500, 0);
  }

  @ParameterizedTest
  @ValueSource(doubles = {1, 2, 3, 4, 5})
  void derivative(double x) throws InterruptedException {
    for (int i = 0; i < 3; i++) {
      Derivative dt = new Derivative();
      dt.calculate(1);
      Thread.sleep(100);
      assertEquals(9.75 * (x - 1), dt.calculate(x), 2);
    }
  }
}
