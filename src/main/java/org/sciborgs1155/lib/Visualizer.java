package org.sciborgs1155.lib;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

/** Visualization class specific for our charged up bot */
public class Visualizer implements Loggable {
  @Log private final Mechanism2d mech;
  private final MechanismRoot2d chassis;
  private final MechanismLigament2d elevatorMax;
  private final MechanismLigament2d elevatorProgress;
  private final MechanismLigament2d forearm;
  private final MechanismLigament2d claw;

  public Visualizer() {
    mech = new Mechanism2d(20, 50);
    chassis = mech.getRoot("Chassis", 10, 0);
    elevatorMax = chassis.append(new MechanismLigament2d("Superstructure", 40, 90));
    elevatorProgress = chassis.append(new MechanismLigament2d("Elevator", 30, 90));
    forearm = elevatorProgress.append(new MechanismLigament2d("Forearm", 10, 30));
    claw = forearm.append(new MechanismLigament2d("Wrist", 10, -10));
  }

  public void setElevatorHeight(double height) {
    elevatorProgress.setLength(height);
  }

  public void setArmAngles(Rotation2d elbow, Rotation2d wrist) {
    forearm.setAngle(elbow);
    claw.setAngle(wrist);
  }
}
