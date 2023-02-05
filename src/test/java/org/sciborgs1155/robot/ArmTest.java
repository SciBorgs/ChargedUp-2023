package org.sciborgs1155.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.*;
import org.sciborgs1155.lib.Visualizer;
import org.sciborgs1155.robot.subsystems.Arm;

public class ArmTest {
  static Visualizer visualizer = new Visualizer();
  static Arm arm = new Arm(visualizer);

  @BeforeEach
  void setup() {
    assert HAL.initialize(500, 0);
  }

  @Test
  void changeElbowGoalTest() throws InterruptedException {
    Rotation2d newElbowGoal = new Rotation2d(2);
    arm.setElbowGoal(newElbowGoal);
    // arm.changeElbowGoal(newElbowGoal).execute();
    assertEquals(newElbowGoal, arm.getElbowgoal());
  }

  @Test
  void changeWristGoalTest() throws InterruptedException {
    Rotation2d newWristGoal = new Rotation2d(4);
    arm.setWristGoal(newWristGoal);
    assertEquals(newWristGoal, arm.getWristGoal());
  }
}
