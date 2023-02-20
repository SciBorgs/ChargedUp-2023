package org.sciborgs1155.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.*;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;
import org.sciborgs1155.robot.Constants.Dimensions;
import org.sciborgs1155.robot.util.Visualizer;

public class ElevatorTest {

  Elevator elevator;

  @BeforeEach
  void setup() {
    assert HAL.initialize(500, 0);
    elevator = new Elevator(new Visualizer());
  }

  @AfterEach
  void destroy() {
    elevator.close();
  }

  @Test
  void setGoal() {
    elevator.setGoal(1).ignoringDisable(true).schedule();
    assertEquals(1, elevator.getGoal());

    elevator.setGoal(Dimensions.ELEVATOR_MAX_HEIGHT + 5).ignoringDisable(true).schedule();
    assertEquals(Dimensions.ELEVATOR_MAX_HEIGHT, elevator.getGoal());

    elevator.setGoal(-3).ignoringDisable(true).schedule();
    assertEquals(Dimensions.ELEVATOR_MIN_HEIGHT, elevator.getGoal());
  }

  @Disabled
  @ParameterizedTest
  @ValueSource(doubles = {2, 3, 6, 9, 39, 40})
  void moveToGoal(double height) {
    elevator.setGoal(height).ignoringDisable(true).schedule();
    for (int i = 0; i < 1000; i++) {
      elevator.periodic();
      elevator.simulationPeriodic();
    }
    assertEquals(height, elevator.getHeight(), 1e-1);
  }
}
