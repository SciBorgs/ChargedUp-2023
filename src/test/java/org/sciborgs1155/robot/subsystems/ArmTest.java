package org.sciborgs1155.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.*;
import org.sciborgs1155.lib.Visualizer;

public class ArmTest {
  static Visualizer visualizer = new Visualizer();
  static Arm arm = new Arm(visualizer);

  @BeforeEach
  void setup() {
    assert HAL.initialize(500, 0);
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
    assertEquals(newWristGoal, arm.getWristGoal());
  }
}
