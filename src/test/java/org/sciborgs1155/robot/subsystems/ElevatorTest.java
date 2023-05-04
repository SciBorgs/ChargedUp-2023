package org.sciborgs1155.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;
import static org.sciborgs1155.robot.Constants.Elevator.*;
import static org.sciborgs1155.robot.TestingUtil.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.junit.jupiter.api.*;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;
import org.sciborgs1155.robot.util.Visualizer;

public class ElevatorTest {

  Elevator elevator;

  static final double delta = 5e-2;

  @BeforeEach
  void setup() {
    setupHAL();
    elevator = new Elevator(Visualizer.basicVisualizer(), Visualizer.basicVisualizer());
    CommandScheduler.getInstance().run();
    // check that it starts at 0
    assertEquals(0, elevator.getPosition(), delta);
    // set setpoint to position
    run(elevator.setSetpoint(elevator.getPosition()));
    assertEquals(elevator.getPosition(), elevator.getSetpoint().position(), delta);
  }

  @AfterEach
  void destroy() throws Exception {
    closeSubsystem(elevator);
  }

  @ParameterizedTest
  @ValueSource(doubles = {-0.2, 0.3, 0.5, 0.7, 0.8})
  void goTo(double setpoint) {
    // set setpoint and test that it was set and clamped correctly
    run(elevator.setSetpoint(setpoint));
    double clamped = MathUtil.clamp(setpoint, MIN_HEIGHT, MAX_HEIGHT);
    assertEquals(clamped, elevator.getSetpoint().position(), "failed to set elevator setpoint");
    fastForward();
    // check that it has reached it's goal
    assertEquals(clamped, elevator.getPosition(), 1e-2, "failed to reach elevator setpoint");
  }

  @Test
  void safety() {
    // tests if it stops moving when stopped
    run(elevator.setStopped(true));
    double position = elevator.getPosition();
    run(elevator.setSetpoint(0.3));
    fastForward();
    assertEquals(position, elevator.getPosition(), delta);
  }
}
