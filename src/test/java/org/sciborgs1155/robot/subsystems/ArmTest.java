package org.sciborgs1155.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.junit.jupiter.api.*;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;
import org.sciborgs1155.lib.ArmState;

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
    var newGoal = ArmState.fromRelative(2, 4);
    arm.setGoal(newGoal).ignoringDisable(true).schedule();
    CommandScheduler.getInstance().run();
    assertEquals(newGoal.elbowAngle(), arm.getGoal().elbowAngle());
    assertEquals(newGoal.wristAngle(), arm.getGoal().wristAngle());
  }

  @Disabled
  @ParameterizedTest
  @ValueSource(doubles = {-0.7, -0.3, 0.0, 0.3, 0.7})
  void moveToGoal(double radGoal) {
    var goal = ArmState.fromRelative(radGoal, 0);
    arm.runToGoal(goal).ignoringDisable(true).schedule();
    for (int i = 0; i < 400; i++) {
      arm.periodic();
      arm.simulationPeriodic();
    }
    assertEquals(radGoal, arm.getState().elbowAngle().getRadians(), 5e-2);
  }
}
