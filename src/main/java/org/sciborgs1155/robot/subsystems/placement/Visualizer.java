package org.sciborgs1155.robot.subsystems.placement;

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

  private static double WEIGHT = 4;
  private static Rotation2d RIGHT_ANGLE = Rotation2d.fromRadians(Math.PI / 2.0);

  private final Mechanism2d mech =
      new Mechanism2d(Dimensions.ARM_LENGTH * 3.0, Dimensions.ELEVATOR_MAX_HEIGHT * 3.0 / 2.0);
  private final MechanismRoot2d chassis =
      mech.getRoot("Chassis", Dimensions.ARM_LENGTH * 3.0 / 2.0, 0);

  private final MechanismLigament2d elevator, forearm, claw;

  public Visualizer(Color8Bit color) {
    elevator =
        chassis.append(
            new MechanismLigament2d(
                "Elevator Position", Dimensions.ELEVATOR_MAX_HEIGHT, 90, WEIGHT, color));
    forearm =
        elevator.append(
            new MechanismLigament2d(
                "Forearm Position", Dimensions.FOREARM_LENGTH, 0, WEIGHT, color));
    claw =
        forearm.append(
            new MechanismLigament2d("Wrist Position", Dimensions.CLAW_LENGTH, 0, WEIGHT, color));
  }

  public void setState(State state) {
    elevator.setLength(state.elevatorHeight());
    forearm.setAngle(state.elbowAngle().minus(RIGHT_ANGLE));
    claw.setAngle(state.wristAngle());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    mech.initSendable(builder);
  }
}
