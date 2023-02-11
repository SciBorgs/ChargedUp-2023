package org.sciborgs1155.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.*;

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
  void setElbowGoalTest() {
    Rotation2d newElbowGoal = new Rotation2d(2);
    arm.setElbowGoal(newElbowGoal).ignoringDisable(true).schedule();
    assertEquals(newElbowGoal, arm.getElbowGoal());
  }

  @Test
  void setWristGoalTest() {
    Rotation2d newWristGoal = new Rotation2d(4);
    arm.setWristGoal(newWristGoal).ignoringDisable(true).schedule();
    assertEquals(newWristGoal, arm.getRelativeWristGoal());
  }

  // @Disabled
  // @ParameterizedTest
  // @ValueSource(doubles = {-0.7, -0.3, 0.0, 0.3, 0.7})
  // void moveToGoal(double radGoal) {
  //   var goal = ArmState.fromRelative(radGoal, 0);
  //   arm.runToGoals(goal).ignoringDisable(true).schedule();
  //   for (int i = 0; i < 400; i++) {
  //     arm.periodic();
  //     arm.simulationPeriodic();
  //   }
  //   assertEquals(radGoal, arm.getState().elbowAngle().getRadians(), 5e-2);
  // }
}
