package org.sciborgs1155.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.*;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

public class ArmTest {
  static Arm arm;

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
    arm.setRelativeWristGoal(newWristGoal).ignoringDisable(true).schedule();
    assertEquals(newWristGoal, arm.getRelativeWristGoal());
  }

  @ParameterizedTest
  @ValueSource(doubles = {-0.7, -0.3, 0.0, 0.3, 0.7})
  void moveWristToRelativeGoal(double radGoal) {
    Rotation2d goal = new Rotation2d(radGoal);
    arm.setRelativeWristGoal(goal).ignoringDisable(true).schedule();
    for (int i = 0; i < 1000; i++) {
      arm.periodic();
      arm.simulationPeriodic();
    }
    assertEquals(goal.getRadians(), arm.getRelativeWristPosition().getRadians(), 5e-2);
  }

  @ParameterizedTest
  @ValueSource(doubles = {-0.7, -0.3, 0.0, 0.3, 0.7})
  void moveWristToAbsoluteGoal(double radGoal) {
    Rotation2d goal = new Rotation2d(radGoal);
    arm.setAbsoluteWristGoal(goal).ignoringDisable(true).schedule();
    for (int i = 0; i < 1000; i++) {
      arm.periodic();
      arm.simulationPeriodic();
    }
    assertEquals(goal.getRadians(), arm.getAbsoluteWristPosition().getRadians(), 9e-2);
  }

  @ParameterizedTest
  @ValueSource(doubles = {-0.7, -0.3, 0.0, 0.3, 0.7})
  void moveElbowToGoal(double radGoal) {
    Rotation2d goal = new Rotation2d(radGoal);
    arm.setElbowGoal(goal).ignoringDisable(true).schedule();
    for (int i = 0; i < 1000; i++) {
      arm.periodic();
      arm.simulationPeriodic();
    }
    assertEquals(goal.getRadians(), arm.getElbowPosition().getRadians(), 9e-3);
  }
}
