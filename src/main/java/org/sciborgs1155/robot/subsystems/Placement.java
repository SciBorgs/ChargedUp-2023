package org.sciborgs1155.robot.subsystems;

import static org.sciborgs1155.robot.Ports.Placement.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import org.sciborgs1155.lib.Derivative;
import org.sciborgs1155.lib.Visualizer;
import org.sciborgs1155.robot.Constants.Arm.ElbowConstants;
import org.sciborgs1155.robot.Constants.Arm.WristConstants;
import org.sciborgs1155.robot.subsystems.placement.Elbow;
import org.sciborgs1155.robot.subsystems.placement.Wrist;

public class Placement extends SubsystemBase implements Loggable, AutoCloseable {
  @Log private final Wrist wrist = Wrist.create(WRIST_MOTOR);

  @Log private final Elbow elbow = Elbow.create(MIDDLE_ELBOW_MOTOR, LEFT_ELBOW_MOTOR, RIGHT_ELBOW_MOTOR);

@Log private final Elevator elevator = Elevator.c

  @Log
  private final ProfiledPIDController wristFeedback =
      new ProfiledPIDController(
          WristConstants.kP, WristConstants.kI, WristConstants.kD, WristConstants.CONSTRAINTS);

  @Log
  private final ProfiledPIDController elbowFeedback =
      new ProfiledPIDController(
          ElbowConstants.kP, ElbowConstants.kI, ElbowConstants.kD, ElbowConstants.CONSTRAINTS);

  private final ArmFeedforward wristFeedforward =
      new ArmFeedforward(
          WristConstants.kS, WristConstants.kG, WristConstants.kV, WristConstants.kA);

  private final ArmFeedforward elbowFeedforward =
      new ArmFeedforward(
          ElbowConstants.kS, ElbowConstants.kG, ElbowConstants.kV, ElbowConstants.kA);

  @Log(name = "wrist acceleration", methodName = "getLastOutput")
  private final Derivative wristAccel = new Derivative();

  @Log(name = "elbow acceleration", methodName = "getLastOutput")
  private final Derivative elbowAccel = new Derivative();

  private final Visualizer visualizer;
}
