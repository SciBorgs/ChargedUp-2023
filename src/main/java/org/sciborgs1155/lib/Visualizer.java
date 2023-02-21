package org.sciborgs1155.lib;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.sciborgs1155.robot.Constants.Dimensions;

/** Visualization class specific for our charged up bot */
public class Visualizer implements Sendable {

  private static Color8Bit POSITION_COLOR = new Color8Bit(255, 0, 0);
  private static Color8Bit SETPOINT_COLOR = new Color8Bit(0, 0, 255);
  private static double WEIGHT = 4;
  private static Rotation2d RIGHT_ANGLE = Rotation2d.fromRadians(Math.PI / 2.0);

  private final Mechanism2d mech =
      new Mechanism2d(Dimensions.ARM_LENGTH * 3.0, Dimensions.ELEVATOR_MAX_HEIGHT * 3.0 / 2.0);
  private final MechanismRoot2d chassis =
      mech.getRoot("Chassis", Dimensions.ARM_LENGTH * 3.0 / 2.0, 0);

  private final MechanismLigament2d elevatorPosition =
      chassis.append(
          new MechanismLigament2d(
              "Elevator Position", Dimensions.ELEVATOR_MAX_HEIGHT, 90, WEIGHT, POSITION_COLOR));
  private final MechanismLigament2d forearmPosition =
      elevatorPosition.append(
          new MechanismLigament2d(
              "Forearm Position", Dimensions.FOREARM_LENGTH, 0, WEIGHT, POSITION_COLOR));
  private final MechanismLigament2d clawPosition =
      forearmPosition.append(
          new MechanismLigament2d(
              "Wrist Position", Dimensions.CLAW_LENGTH, 0, WEIGHT, POSITION_COLOR));

  private final MechanismLigament2d elevatorSetpoint =
      chassis.append(
          new MechanismLigament2d(
              "Elevator Setpoint", Dimensions.ELEVATOR_MAX_HEIGHT, 90, WEIGHT, SETPOINT_COLOR));
  private final MechanismLigament2d forearmSetpoint =
      elevatorSetpoint.append(
          new MechanismLigament2d(
              "Forearm Setpoint", Dimensions.FOREARM_LENGTH, 0, WEIGHT, SETPOINT_COLOR));
  private final MechanismLigament2d clawSetpoint =
      forearmSetpoint.append(
          new MechanismLigament2d(
              "Wrist Setpoint", Dimensions.CLAW_LENGTH, 0, WEIGHT, SETPOINT_COLOR));

  public void setPositions(PlacementState state) {
    elevatorPosition.setLength(state.elevatorHeight());
    forearmPosition.setAngle(state.elbowAngle().minus(RIGHT_ANGLE));
    clawPosition.setAngle(state.wristAngle());
  }

  public void setSetpoints(PlacementState state) {
    elevatorSetpoint.setLength(state.elevatorHeight());
    forearmSetpoint.setAngle(state.elbowAngle().minus(RIGHT_ANGLE));
    clawSetpoint.setAngle(state.wristAngle());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    mech.initSendable(builder);
  }
}
