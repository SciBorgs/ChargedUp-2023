// package org.sciborgs1155.robot.subsystems;

// import static org.junit.jupiter.api.Assertions.*;
// import static org.sciborgs1155.robot.TestingUtil.*;

// import edu.wpi.first.hal.HAL;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.util.Color;
// import edu.wpi.first.wpilibj.util.Color8Bit;

// import java.util.stream.Stream;
// import org.junit.jupiter.api.*;
// import org.junit.jupiter.params.ParameterizedTest;
// import org.junit.jupiter.params.provider.Arguments;
// import org.junit.jupiter.params.provider.MethodSource;
// import org.junit.jupiter.params.provider.ValueSource;
// import org.sciborgs1155.robot.Constants;
// import org.sciborgs1155.robot.util.Visualizer;

// public class ArmTest {
//   Arm arm;

//   @BeforeEach
//   void setup() {
//     assert HAL.initialize(500, 0);
//     arm = new Arm(Visualizer.basicVisualizer(), Visualizer.basicVisualizer());
//   }

//   @AfterEach
//   void reset() {
//     closeSubsystem(arm);
//   }

//   @Test
//   void elbowGoTo() {
//     // set elbow setpoint
//     assertEquals(arm.getElbowPosition(), Constants.Arm.Elbow.ELBOW_OFFSET);
//     run(arm.setSetpoints(arm.getElbowPosition(), arm.getRelativeWristPosition()));
//   }

//   @Test
//   void setWristGoalTest() {
//     Rotation2d newWristGoal = new Rotation2d(1.3);
//     arm.setWristGoal(newWristGoal).ignoringDisable(true).schedule();
//     assertEquals(newWristGoal, arm.getRelativeWristGoal());
//   }

//   @Disabled
//   @ParameterizedTest
//   @ValueSource(doubles = {-0.7, -0.3, 0.0, 0.3, 0.7})
//   void moveElbowToGoal(double radGoal) {
//     Rotation2d goal = new Rotation2d(radGoal);
//     arm.runToGoals(goal, new Rotation2d(0)).ignoringDisable(true).schedule();
//     for (int i = 0; i < 400; i++) {
//       arm.periodic();
//       arm.simulationPeriodic();
//     }
//     assertEquals(radGoal, arm.getElbowPosition().getRadians(), 5e-2);
//   }

//   @Disabled
//   @ParameterizedTest
//   @ValueSource(doubles = {-0.7, -0.3, 0.0, 0.3, 0.7})
//   void moveWristToGoal(double radGoal) {
//     Rotation2d goal = new Rotation2d(radGoal);
//     arm.runWristToGoal(goal).ignoringDisable(true).schedule();
//     for (int i = 0; i < 1000; i++) {
//       arm.periodic();
//       arm.simulationPeriodic();
//     }
//     assertEquals(radGoal, arm.getRelativeWristPosition().getRadians(), 5e-2);
//   }

//   @Disabled
//   @ParameterizedTest
//   @MethodSource
//   void moveToGoal(double elbowRadGoal, double wristRadGoal) {
//     Rotation2d elbowGoal = new Rotation2d(elbowRadGoal);
//     Rotation2d wristGoal = new Rotation2d(wristRadGoal);
//     arm.runToGoals(elbowGoal, wristGoal).ignoringDisable(true).schedule();
//     for (int i = 0; i < 1000; i++) {
//       arm.periodic();
//       arm.simulationPeriodic();
//     }
//     assertEquals(elbowRadGoal, arm.getElbowPosition().getRadians(), 5e-2);
//     assertEquals(wristRadGoal, arm.getRelativeWristPosition().getRadians(), 5e-2);
//   }

//   static Stream<Arguments> moveToGoal() {
//     return Stream.of(
//         Arguments.arguments(-1, -0.8),
//         Arguments.arguments(-1, 0.7),
//         Arguments.arguments(0, 0),
//         Arguments.arguments(0.3, 1));
//   }
// }
