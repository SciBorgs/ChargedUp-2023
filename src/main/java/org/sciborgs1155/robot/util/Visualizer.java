package org.sciborgs1155.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import org.sciborgs1155.robot.Constants.Dimensions;

/** Visualization class specific for our charged up bot */
public class Visualizer implements Sendable {

  private final Mechanism2d mech = new Mechanism2d(2, 4);
  private final MechanismRoot2d chassis = mech.getRoot("Chassis", 1, 2);
  private final MechanismLigament2d elevatorProgress =
      chassis.append(new MechanismLigament2d("Elevator", Dimensions.ELEVATOR_MAX_HEIGHT, 90));
  private final MechanismLigament2d forearm =
      elevatorProgress.append(new MechanismLigament2d("Forearm", Dimensions.FOREARM_LENGTH, 0));
  private final MechanismLigament2d claw =
      forearm.append(new MechanismLigament2d("Wrist", Dimensions.CLAW_LENGTH, 0));

  public Visualizer() {
    // chassis.append(new MechanismLigament2d("Superstructure", Dimensions.ELEVATOR_MAX_HEIGHT,
    // 90));
  }

  public void setElevatorHeight(double height) {
    elevatorProgress.setLength(height);
  }

  public void setArmPositions(Rotation2d elbowAngle, Rotation2d wristAngle) {
    forearm.setAngle(elbowAngle);
    claw.setAngle(wristAngle);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    mech.initSendable(builder);
  }
}
