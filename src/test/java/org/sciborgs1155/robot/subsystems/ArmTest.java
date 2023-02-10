package org.sciborgs1155.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.*;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

public class ArmTest {
  Arm arm;

  @BeforeEach
  void setup() {
    assert HAL.initialize(500, 0);
    arm = new Arm();
  }

  @AfterEach
  void reset() {
    arm.close();
  }

  @Test
  void setGoal() {
    var newGoal = Arm.State.fromRelative(2, 4);
    arm.setGoal(newGoal).ignoringDisable(true).schedule();
    // assertEquals(newGoal, arm.getGoal());
  }

  @ParameterizedTest
  @ValueSource(doubles = {-0.7, -0.3, 0.0, 0.3, 0.7})
  void moveToGoal(double radGoal) {
    var goal = Arm.State.fromRelative(radGoal, 0);
    arm.runToGoal(goal).ignoringDisable(true).schedule();
    for (int i = 0; i < 400; i++) {
      arm.periodic();
      arm.simulationPeriodic();
    }
    // assertEquals(goal, arm.getElbowPosition().getRadians(), 5e-2);
  }
}
