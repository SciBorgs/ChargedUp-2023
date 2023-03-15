package org.sciborgs1155.lib;

import org.junit.jupiter.api.Test;
import org.sciborgs1155.lib.constants.Conversion;
import org.sciborgs1155.lib.constants.Conversion.PulsesPerRev;
import org.sciborgs1155.lib.constants.Conversion.Units;

public class ConfigTests {

  @Test
  void conversion() {
    double radius = 0.0181864;
    double conversion = 2.0 * Math.PI * radius; // m
    double factor = conversion / 2048.0; // throughbore cpr;

    Conversion ratio =
        Conversion.base()
            .multiplyRadius(0.0181864)
            .withUnits(Units.RADIANS)
            .withPulsesPerRev(PulsesPerRev.REV_THROUGHBORE);

    assert factor == ratio.factor();
  }
}
