package org.sciborgs1155.lib;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import org.sciborgs1155.robot.Constants.Dimensions;

/** Visualization class specific for our charged up bot */
public class Visualizer implements Sendable {
  private final Mechanism2d mech;
  private final MechanismRoot2d chassis;
  private final MechanismLigament2d elevatorProgress;
  private final MechanismLigament2d forearm;
  private final MechanismLigament2d claw;

  public Visualizer() {
    mech = new Mechanism2d(20, 50);
    chassis = mech.getRoot("Chassis", 10, 0);
    chassis.append(new MechanismLigament2d("Superstructure", Dimensions.ELEVATOR_HEIGHT, 90));
    elevatorProgress =
        chassis.append(new MechanismLigament2d("Elevator", Dimensions.ELEVATOR_HEIGHT / 2, 90));
    forearm =
        elevatorProgress.append(new MechanismLigament2d("Forearm", Dimensions.FOREARM_LENGTH, 30));
    claw = forearm.append(new MechanismLigament2d("Wrist", Dimensions.CLAW_LENGTH, -10));
  }

  public void setElevatorHeight(double height) {
    elevatorProgress.setLength(height);
  }

  public void setArmAngles(Rotation2d elbow, Rotation2d wrist) {
    forearm.setAngle(elbow);
    claw.setAngle(wrist);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    mech.initSendable(builder);
  }
}
