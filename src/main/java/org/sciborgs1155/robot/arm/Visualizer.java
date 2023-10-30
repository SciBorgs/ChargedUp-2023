package org.sciborgs1155.robot.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.sciborgs1155.robot.Constants.Dimensions;
import org.sciborgs1155.robot.arm.ArmConstants.Elevator;

/** Visualization class for our charged up bot's arm */
public class Visualizer {

  private static double WEIGHT = 4;
  private static int instance = 0;
  private static Rotation2d RIGHT_ANGLE = Rotation2d.fromRadians(Math.PI / 2.0);

  private final MechanismRoot2d chassis;
  private final MechanismLigament2d elevator, forearm, claw;

  public Visualizer(Mechanism2d mech, Color8Bit color) {
    chassis =
        mech.getRoot("Chassis " + instance, 2 + Dimensions.BASE_OFFSET, Dimensions.BASE_HEIGHT);
    elevator =
        chassis.append(
            new MechanismLigament2d(
                "Elevator " + instance + " Position", Elevator.MAX_HEIGHT, 90, WEIGHT, color));
    forearm =
        elevator.append(
            new MechanismLigament2d(
                "Forearm " + instance + " Position", Dimensions.FOREARM_LENGTH, 90, WEIGHT, color));
    claw =
        forearm.append(
            new MechanismLigament2d(
                "Wrist " + instance + " Position", Dimensions.CLAW_LENGTH, 0, WEIGHT, color));

    instance++;
  }

  public void setState(ArmState state) {
    elevator.setLength(state.elevatorHeight());
    forearm.setAngle(state.elbowAngle().minus(RIGHT_ANGLE));
    claw.setAngle(state.wristAngle());
  }
}
