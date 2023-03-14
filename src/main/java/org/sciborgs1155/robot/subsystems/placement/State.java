package org.sciborgs1155.robot.subsystems.placement;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N6;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import org.sciborgs1155.robot.Constants.Dimensions;

/** ArmState class to store relative angles for the arm. */
public record State(
    double elevatorHeight,
    Rotation2d elbowAngle,
    Rotation2d wristAngle) {

  /** Represents the side of the robot the arm is on */
  public enum Side {
    FRONT,
    BACK;
  }

  /**
   * Returns a new {@link State} from angles in radians, with the wrist state relative to
   * the chassis
   */
  public static State fromAbsolute(
      double elevatorHeight, double elbowAngle, double wristAngle) {
    return new State(
        elevatorHeight,
        Rotation2d.fromRadians(elbowAngle),
        Rotation2d.fromRadians(wristAngle - elbowAngle));
  }

  /**
   * Returns a new {@link State} from angles in radians, with the wrist state relative to
   * the forearm
   */
  public static State fromRelative(
      double elevatorHeight, double elbowAngle, double wristAngle) {
    return new State(
        elevatorHeight, Rotation2d.fromRadians(elbowAngle), Rotation2d.fromRadians(wristAngle));
  }

  /** Creates a PlacementState from a 6d vector */
  public static State fromVec(Vector<N3> state) {
    return new State(
        state.get(0, 0),
        Rotation2d.fromRadians(state.get(1, 0)),
        Rotation2d.fromRadians(state.get(2, 0)));
  }

  /** Returns a 6d vector representation of the state */
  public Vector<N3> toVec() {
    return VecBuilder.fill(
        elevatorHeight,
        elbowAngle.getRadians(),
        wristAngle.getRadians());
  }

  /** The side of the robot the arm is on */
  public Side side() {
    return elbowAngle.getCos() > 0 ? Side.FRONT : Side.BACK;
  }

  /**
   * Returns the corresponding arm state given a target Math can be found {@link
   * https://robotacademy.net.au/lesson/inverse-kinematics-for-a-2-joint-robot-arm-using-geometry}
   */
  public static State fromIK(Translation3d target, double height) {
    double wristAngle =
        -Math.acos(
            (Math.pow(target.getY(), 2)
                    + Math.pow(target.getZ() - height, 2)
                    - Math.pow(Dimensions.FOREARM_LENGTH, 2)
                    - Math.pow(Dimensions.CLAW_LENGTH, 2))
                / 2.0
                * Dimensions.CLAW_LENGTH
                * Dimensions.FOREARM_LENGTH);
    double elbowAngle =
        Math.atan2(target.getZ() - height, target.getY())
            + Math.atan2(
                Dimensions.CLAW_LENGTH * Math.sin(wristAngle),
                Dimensions.FOREARM_LENGTH + Dimensions.CLAW_LENGTH * Math.cos(wristAngle));
    return fromAbsolute(elbowAngle, wristAngle, 0);
  }

  public boolean roughlyEquals(State other, double margin) {
    var v1 = this.toVec();
    var v2 = other.toVec();
    for (int i = 0; i < v1.getNumCols(); i++) {
      if (Math.abs(v1.get(i, 0) - v2.get(i, 0)) < margin) {
        return false;
      }
    }
    return true;
  }
}
