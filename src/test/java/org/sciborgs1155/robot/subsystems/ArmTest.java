package org.sciborgs1155.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;
import static org.sciborgs1155.robot.Constants.Arm.*;
import static org.sciborgs1155.robot.TestingUtil.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.stream.Stream;
import org.junit.jupiter.api.*;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.junit.jupiter.params.provider.ValueSource;
import org.sciborgs1155.robot.util.Visualizer;

public class ArmTest {
  Arm arm;

  static final double ELBOW_DELTA = 3e-2;
  static final double WRIST_DELTA = 2e-1;

  @BeforeEach
  void setup() {
    setupHAL();
    arm = new Arm(Visualizer.basicVisualizer(), Visualizer.basicVisualizer());
    CommandScheduler.getInstance().run();
    assertEquals(0, arm.getElbowPosition().getRadians(), ELBOW_DELTA);
    assertEquals(0, arm.getRelativeWristPosition().getRadians(), WRIST_DELTA);
  }

  @AfterEach
  void reset() {
    closeSubsystem(arm);
  }

  @ParameterizedTest
  @ValueSource(doubles = {Elbow.MIN_ANGLE, -1, 0, 1, 1.8, 3, Elbow.MAX_ANGLE})
  void elbowGoTo(double setpoint) {
    // set elbow setpoint
    run(arm.setSetpoints(Rotation2d.fromRadians(setpoint), arm.getRelativeWristPosition()));
    assertEquals(setpoint, arm.getElbowSetpoint().position());

    // run to goal
    fastForward();
    assertEquals(setpoint, arm.getElbowPosition().getRadians(), ELBOW_DELTA);
  }

  @ParameterizedTest
  @ValueSource(doubles = {Wrist.MIN_ANGLE, -2, -1, 0, 1, 2, Wrist.MAX_ANGLE})
  void wristGoTo(double setpoint) {
    // check that wrist is not limp
    assertTrue(arm.allowPassOver(), "not allowing passover");

    // set wrist setpoint
    run(arm.setSetpoints(arm.getElbowPosition(), Rotation2d.fromRadians(setpoint)));
    assertEquals(setpoint, arm.getWristSetpoint().position());

    // run to goal
    fastForward();
    assertEquals(setpoint, arm.getRelativeWristPosition().getRadians(), WRIST_DELTA);
  }

  @ParameterizedTest
  @MethodSource("factory")
  void goTo(double elbowSetpoint, double wristSetpoint) {
    // check that wrist is not limp
    assertTrue(arm.allowPassOver(), "wrist is");

    // set setpoints
    run(
        arm.setSetpoints(
            Rotation2d.fromRadians(elbowSetpoint), Rotation2d.fromRadians(wristSetpoint)));
    assertEquals(elbowSetpoint, arm.getElbowSetpoint().position());
    assertEquals(wristSetpoint, arm.getWristSetpoint().position());

    // run to goal
    fastForward();
    assertEquals(elbowSetpoint, arm.getElbowPosition().getRadians(), ELBOW_DELTA);
    assertEquals(wristSetpoint, arm.getRelativeWristPosition().getRadians(), WRIST_DELTA);
  }

  static Stream<Arguments> factory() {
    return Stream.of(
        Arguments.of(Elbow.MIN_ANGLE, Wrist.MIN_ANGLE),
        Arguments.of(0, -2),
        Arguments.of(2, 1),
        Arguments.of(Elbow.MAX_ANGLE, Wrist.MAX_ANGLE));
  }

  void safety() {
    arm.setStopped(true);
    double elbowPos = arm.getElbowPosition().getRadians();
    arm.setSetpoints(Rotation2d.fromRadians(0.4), Rotation2d.fromRadians(0.8));
    fastForward();
    assertEquals(elbowPos, arm.getElbowPosition().getRadians(), 1.5e-1);
  }
}
