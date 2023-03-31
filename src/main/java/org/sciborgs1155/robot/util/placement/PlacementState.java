package org.sciborgs1155.robot.util.placement;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
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

  /** Creates a PlacementState from an absolute 3d vector */
  public static PlacementState fromVec(Vector<N3> state) {
    return fromAbsolute(state.get(0, 0), state.get(1, 0), state.get(2, 0));
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

    // check for nan
    if (elbowAngle != elbowAngle || wristAngle != wristAngle) {
      return Optional.empty();
    }

    // add elevator height if necessary
    double totalHeight =
        Math.sin(elbowAngle) * Dimensions.FOREARM_LENGTH
            + Math.sin(wristAngle) * Dimensions.CLAW_LENGTH;

    double elevatorHeight = totalHeight < target.getY() ? target.getY() - totalHeight : 0;

    var state = fromRelative(elevatorHeight, elbowAngle, wristAngle);
    System.out.println(state);
    System.out.println("claw pos" + state.endEffectorPosition());
    return Optional.of(fromRelative(elevatorHeight, elbowAngle, wristAngle));
  }

  public static Optional<PlacementState> fromIK(Pose2d target) {
    double wristAngle = target.getRotation().getRadians();

    double elbowAngle =
        Math.atan2(target.getY(), target.getX())
            + Math.atan2(
                Dimensions.CLAW_LENGTH * Math.sin(target.getRotation().getRadians()),
                Dimensions.FOREARM_LENGTH + Dimensions.CLAW_LENGTH * Math.cos(wristAngle));

    // check for nan
    if (elbowAngle != elbowAngle || wristAngle != wristAngle) {
      return Optional.empty();
    }

    // add elevator height if necessary
    double totalHeight =
        Math.sin(elbowAngle) * Dimensions.FOREARM_LENGTH
            + Math.sin(wristAngle) * Dimensions.CLAW_LENGTH;

    double elevatorHeight = totalHeight < target.getY() ? target.getY() - totalHeight : 0;

    return Optional.of(fromRelative(elevatorHeight, elbowAngle, wristAngle));
  }

  /** Performs forward kinematics to return the claw's position as a {@link Translation2d} */
  public Translation2d endEffectorPosition() {
    return new Translation2d(0, elevatorHeight)
        .plus(new Translation2d(Dimensions.FOREARM_LENGTH, elbowAngle))
        .plus(new Translation2d(Dimensions.CLAW_LENGTH, wristAngle));
  }

  /** Compares the elevator height, elbow angle, and wrist angle given a margin */
  public boolean roughlyEquals(PlacementState other, double margin) {
    return margin < Math.abs(other.elevatorHeight - this.elevatorHeight)
        ? margin < Math.abs(other.elbowAngle.getRadians() - this.elbowAngle.getRadians())
            ? margin < Math.abs(other.wristAngle.getRadians() - this.wristAngle.getRadians())
            : false
        : false;
  }

  /** Compares the end effector positions of Placement States, given a margin */
  public boolean endRoughlyEquals(PlacementState other, double margin) {
    return Math.sqrt(
            Math.pow(this.endEffectorPosition().getX() - other.endEffectorPosition().getX(), 2)
                + Math.pow(
                    this.endEffectorPosition().getY() - other.endEffectorPosition().getY(), 2))
        < margin;
  }

  public String toString() {
    return "("
        + elevatorHeight
        + ", "
        + elbowAngle.getRadians()
        + ", "
        + wristAngle.getRadians()
        + ")";
  }
}
