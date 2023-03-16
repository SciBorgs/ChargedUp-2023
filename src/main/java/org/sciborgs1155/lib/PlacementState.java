package org.sciborgs1155.lib;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import java.util.Optional;
import org.sciborgs1155.robot.Constants.Dimensions;

/** ArmState class to store relative angles for the arm. */
public record PlacementState(double elevatorHeight, Rotation2d elbowAngle, Rotation2d wristAngle) {

  /** Represents the side of the robot the arm is on */
  public enum Side {
    FRONT,
    BACK;
  }

  /**
   * Returns a new {@link PlacementState} from angles in radians, with the wrist state relative to
   * the chassis
   */
  public static PlacementState fromAbsolute(
      double elevatorHeight, double elbowAngle, double wristAngle) {
    return new PlacementState(
        elevatorHeight,
        Rotation2d.fromRadians(elbowAngle),
        Rotation2d.fromRadians(wristAngle - elbowAngle));
  }

  /**
   * Returns a new {@link PlacementState} from angles in radians, with the wrist state relative to
   * the forearm
   */
  public static PlacementState fromRelative(
      double elevatorHeight, double elbowAngle, double wristAngle) {
    return new PlacementState(
        elevatorHeight, Rotation2d.fromRadians(elbowAngle), Rotation2d.fromRadians(wristAngle));
  }

  /** Creates a PlacementState from a 6d vector */
  public static PlacementState fromVec(Vector<N3> state) {
    return new PlacementState(
        state.get(0, 0),
        Rotation2d.fromRadians(state.get(1, 0)),
        Rotation2d.fromRadians(state.get(2, 0)));
  }

  /** Returns a 6d vector representation of the state */
  public Vector<N3> toVec() {
    return VecBuilder.fill(elevatorHeight, elbowAngle.getRadians(), wristAngle.getRadians());
  }

  /** The side of the robot the arm is on */
  public Side side() {
    return elbowAngle.getCos() > 0 ? Side.FRONT : Side.BACK;
  }

  /**
   * Returns the corresponding arm state given a target Math can be found {@link
   * https://robotacademy.net.au/lesson/inverse-kinematics-for-a-2-joint-robot-arm-using-geometry}
   */
  public static Optional<PlacementState> fromIK(Translation2d target) {
    // get initial wrist and elbow angles
    double wristAngle =
        -Math.acos(
            (Math.pow(target.getY(), 2)
                    + Math.pow(target.getX(), 2)
                    - Math.pow(Dimensions.FOREARM_LENGTH, 2)
                    - Math.pow(Dimensions.CLAW_LENGTH, 2))
                / (2.0 * Dimensions.CLAW_LENGTH * Dimensions.FOREARM_LENGTH));
    double elbowAngle =
        Math.atan2(target.getY(), target.getX())
            + Math.atan2(
                Dimensions.CLAW_LENGTH * Math.sin(wristAngle),
                Dimensions.FOREARM_LENGTH + Dimensions.CLAW_LENGTH * Math.cos(wristAngle));

    // add elevator height if necessary
    double currentHeight =
        Math.sin(elbowAngle) * Dimensions.FOREARM_LENGTH
            + Math.sin(wristAngle) * Dimensions.CLAW_LENGTH;

    double elevatorHeight = 0;
    // if (currentHeight < target.getY()) {
    //   elevatorHeight = target.getY() - currentHeight;
    // }

    // check to see if state is valid
    if (elevatorHeight != elevatorHeight || elbowAngle != elbowAngle || wristAngle != wristAngle) {
      return Optional.empty();
    }

    var state = fromAbsolute(elevatorHeight, elbowAngle, wristAngle);
    System.out.println(state);
    return Optional.of(fromAbsolute(elevatorHeight, elbowAngle, wristAngle));
  }

  public boolean roughlyEquals(PlacementState other, double margin) {
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
