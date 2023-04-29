package org.sciborgs1155.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.sciborgs1155.robot.Constants.Dimensions;
import org.sciborgs1155.robot.Constants.Elevator;

/** Visualization class specific for our charged up bot */
public class Visualizer implements Sendable {

  private static double WEIGHT = 4;
  private static Rotation2d RIGHT_ANGLE = Rotation2d.fromRadians(Math.PI / 2.0);

  private final Mechanism2d mech = new Mechanism2d(4, 2);
  private final MechanismRoot2d chassis =
      mech.getRoot("Chassis", 2 + Dimensions.BASE_OFFSET, Dimensions.BASE_HEIGHT);

  private final MechanismLigament2d elevator, forearm, claw;

  public Visualizer(Color8Bit color) {
    elevator =
        chassis.append(
            new MechanismLigament2d("Elevator Position", Elevator.MAX_HEIGHT, 90, WEIGHT, color));
    forearm =
        elevator.append(
            new MechanismLigament2d(
                "Forearm Position", Dimensions.FOREARM_LENGTH, 90, WEIGHT, color));
    claw =
        forearm.append(
            new MechanismLigament2d("Wrist Position", Dimensions.CLAW_LENGTH, 0, WEIGHT, color));
  }

  /**
   * returns a new visualizer with a defualt color (black) use if color doesn't matter (i.e. unit
   * tests)
   */
  public static Visualizer basicVisualizer() {
    return new Visualizer(new Color8Bit(Color.kBlack));
  }

  public void setArmAngles(Rotation2d elbowAngle, Rotation2d wristAngle) {
    forearm.setAngle(elbowAngle.minus(RIGHT_ANGLE));
    claw.setAngle(wristAngle);
  }

  public void setElevatorHeight(double height) {
    elevator.setLength(height);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    mech.initSendable(builder);
  }
}
