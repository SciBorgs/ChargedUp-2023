package org.sciborgs1155.lib;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import org.sciborgs1155.robot.Constants.Dimensions;

/** ArmState class to store relative angles for the arm. */
public record ArmState(Rotation2d elbowAngle, Rotation2d wristAngle, double elevatorHeight)
    implements Sendable {

  /** Represents the side of the robot the arm is on */
  public enum Side {
    FRONT,
    BACK;
  }

  /**
   * Returns a new {@link ArmState} from angles in radians, with the wrist state relative to the
   * chassis
   */
  public static ArmState fromAbsolute(double elbowAngle, double wristAngle, double elevatorHeight) {
    return new ArmState(
        Rotation2d.fromRadians(elbowAngle),
        Rotation2d.fromRadians(wristAngle - elbowAngle),
        elevatorHeight);
  }

  /**
   * Returns a new {@link ArmState} from angles in radians, with the wrist state relative to the
   * forearm
   */
  public static ArmState fromRelative(double elbowAngle, double wristAngle, double elevatorHeight) {
    // System.out.println("elbow: " + elbowAngle);
    // System.out.println("wrist: " + Rotation2d.fromRadians(wristAngle));
    return new ArmState(
        Rotation2d.fromRadians(elbowAngle), Rotation2d.fromRadians(wristAngle), elevatorHeight);
  }

  /** The side of the robot the arm is on */
  public Side side() {
    return elbowAngle.getCos() > 0 ? Side.FRONT : Side.BACK;
  }

  /**
   * Returns the corresponding arm state given a target Math can be found {@link
   * https://robotacademy.net.au/lesson/inverse-kinematics-for-a-2-joint-robot-arm-using-geometry}
   */
  public static ArmState fromIK(Translation3d target) {
    double wristAngle =
        -Math.acos(
            (Math.pow(target.getY(), 2)
                    + Math.pow(target.getZ(), 2)
                    - Math.pow(Dimensions.FOREARM_LENGTH, 2)
                    - Math.pow(Dimensions.CLAW_LENGTH, 2))
                / 2.0
                * Dimensions.CLAW_LENGTH
                * Dimensions.FOREARM_LENGTH);
    double elbowAngle =
        Math.atan2(target.getZ(), target.getY())
            + Math.atan2(
                Dimensions.CLAW_LENGTH * Math.sin(wristAngle),
                Dimensions.FOREARM_LENGTH + Dimensions.CLAW_LENGTH * Math.cos(wristAngle));
    return fromAbsolute(elbowAngle, wristAngle, 0);
  }

  public double elbowAngleRadians() {
    return elbowAngle().getRadians();
  }

  public double wristAngleRadians() {
    return wristAngle().getRadians();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("elbow angle", () -> elbowAngle().getDegrees(), null);
    builder.addDoubleProperty("relative wrist angle", () -> wristAngle().getDegrees(), null);
    builder.addDoubleProperty(
        "absolute wrist angle", () -> elbowAngle().plus(wristAngle()).getDegrees(), null);
    builder.addDoubleProperty("elevator height", this::elevatorHeight, null);
  }
}
