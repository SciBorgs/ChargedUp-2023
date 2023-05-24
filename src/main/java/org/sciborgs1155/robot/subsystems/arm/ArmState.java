package org.sciborgs1155.robot.subsystems.arm;

import static org.sciborgs1155.robot.Constants.Positions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Optional;
import org.sciborgs1155.robot.Constants.Dimensions;

/** ArmState class to store relative angles for the arm. */
public record ArmState(double elevatorHeight, Rotation2d elbowAngle, Rotation2d wristAngle) {

  /** Represents the side of the robot the arm is on */
  public enum Side {
    FRONT,
    BACK;

    public double rads() {
      if (this == BACK) {
        return Math.PI;
      }
      return 0;
    }
  }

  /** Represents the game piece the robot is holding */
  public enum GamePiece {
    CONE,
    CUBE;
  }

  /** Represents the part of the game the robot is interacting with */
  public enum Level {
    HIGH,
    MID,
    LOW,
    SINGLE_SUBSTATION,
    DOUBLE_SUBSTATION,
  }

  /**
   * Returns a new {@link ArmState} from angles in radians, with the wrist state relative to the
   * chassis
   */
  public static ArmState fromAbsolute(double elevatorHeight, double elbowAngle, double wristAngle) {
    return new ArmState(
        elevatorHeight,
        Rotation2d.fromRadians(elbowAngle),
        Rotation2d.fromRadians(wristAngle - elbowAngle));
  }

  /**
   * Returns a new {@link ArmState} from angles in radians, with the wrist state relative to the
   * forearm
   */
  public static ArmState fromRelative(double elevatorHeight, double elbowAngle, double wristAngle) {
    return new ArmState(
        elevatorHeight, Rotation2d.fromRadians(elbowAngle), Rotation2d.fromRadians(wristAngle));
  }

  /** Creates a PlacementState from an absolute 3d array */
  public static ArmState fromArray(double[] state) {
    return fromAbsolute(state[0], state[1], state[2]);
  }

  /** Returns an array representation of the absolute state */
  public double[] toArray() {
    return new double[] {
      elevatorHeight, elbowAngle.getRadians(), wristAngle.getRadians() + elbowAngle.getRadians()
    };
  }

  /** The side of the robot the arm is on */
  public Side side() {
    return elbowAngle.getCos() > 0 ? Side.FRONT : Side.BACK;
  }

  /**
   * Returns a goal based on the side being moved to. This is used for on the fly trapezoidal
   * control.
   *
   * @param side The side of the robot being moved to.
   * @return The "passing over" goal according to the inputted side.
   */
  public static ArmState passOverToSide(Side side) {
    return side == Side.FRONT ? PASS_TO_FRONT : PASS_TO_BACK;
  }

  /** Returns a PlacementState from scoring parameters */
  public static ArmState fromOperator(Level height, GamePiece gamePiece, Side side) {
    return switch (height) {
        // case LOW -> side == Side.FRONT ? FRONT_INTAKE : BACK_INTAKE;
      case LOW -> switch (gamePiece) {
        case CONE -> GROUND_CONE_INTAKE;
        case CUBE -> GROUND_CONE_INTAKE;
      };
      case MID -> switch (gamePiece) {
        case CONE -> side == Side.FRONT ? FRONT_MID_CONE : BACK_MID_CONE;
        case CUBE -> side == Side.FRONT
            ? FRONT_MID_CUBE
            : BACK_MID_CUBE; // no cube intake yet, here so no errors
      };
      case HIGH -> switch (gamePiece) {
        case CONE -> BACK_HIGH_CONE;
        case CUBE -> side == Side.FRONT ? FRONT_HIGH_CUBE : BACK_HIGH_CUBE;
      };
      case SINGLE_SUBSTATION -> switch (gamePiece) {
        case CONE -> FRONT_SINGLE_SUBSTATION_CONE;
        case CUBE -> FRONT_SINGLE_SUBSTATION_CUBE;
      };
      case DOUBLE_SUBSTATION -> BACK_DOUBLE_SUBSTATION;
    };
  }

  /**
   * Returns the corresponding arm state given a target Math can be found {@link
   * https://robotacademy.net.au/lesson/inverse-kinematics-for-a-2-joint-robot-arm-using-geometry}
   */
  public static Optional<ArmState> fromIK(Translation2d target) {
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

    return Optional.of(fromRelative(elevatorHeight, elbowAngle, wristAngle));
  }

  public static Optional<ArmState> fromIK(Pose2d target) {
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
  public boolean roughlyEquals(ArmState other, double margin) {
    return margin < Math.abs(other.elevatorHeight - this.elevatorHeight)
        ? margin < Math.abs(other.elbowAngle.getRadians() - this.elbowAngle.getRadians())
            ? margin < Math.abs(other.wristAngle.getRadians() - this.wristAngle.getRadians())
            : false
        : false;
  }

  /** Compares the end effector positions of Placement States, given a margin */
  public boolean endRoughlyEquals(ArmState other, double margin) {
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
