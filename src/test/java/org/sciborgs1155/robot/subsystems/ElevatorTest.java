package org.sciborgs1155.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.*;
import org.sciborgs1155.robot.Constants.Dimensions;

public class ElevatorTest {

  Elevator elevator = new Elevator();

  @BeforeEach
  void setup() {
    assert HAL.initialize(500, 0);
  }

  @Test
  void setGoal() {
    elevator.setGoal(9).ignoringDisable(true).schedule();
    assertEquals(9, elevator.getGoal());

    elevator.setGoal(Dimensions.ELEVATOR_MAX_HEIGHT + 5).ignoringDisable(true).schedule();
    assertEquals(Dimensions.ELEVATOR_MAX_HEIGHT, elevator.getGoal());

    elevator.setGoal(-3).ignoringDisable(true).schedule();
    assertEquals(Dimensions.ELEVATOR_MIN_HEIGHT, elevator.getGoal());
  }
}
