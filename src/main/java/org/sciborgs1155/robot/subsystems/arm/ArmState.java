package org.sciborgs1155.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Optional;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Constants.Dimensions;
import org.sciborgs1155.robot.Constants.Elbow;
import org.sciborgs1155.robot.Constants.Elevator;
import org.sciborgs1155.robot.Constants.RobotType;

/** ArmState class to store relative angles for the arm. */
public record ArmState(double elevatorHeight, Rotation2d elbowAngle, Rotation2d wristAngle) {

  public static final ArmState OLD_INITIAL =
      ArmState.fromRelative(Elevator.ZERO_OFFSET, Elbow.OFFSET, Math.PI);

  public static final ArmState NEW_INITIAL =
      ArmState.fromRelative(Elevator.ZERO_OFFSET, Elbow.OFFSET, 2.4);

  public static final ArmState STOW = ArmState.fromRelative(0, 1.21834, Math.PI / 2.0);

  // LOWEST COG
  public static final ArmState OLD_SAFE =
      ArmState.fromAbsolute(Elevator.ZERO_OFFSET, Elbow.OFFSET + 0.1, Math.PI / 2);

  public static final ArmState PASS_TO_BACK = ArmState.fromAbsolute(0.03, Math.PI / 2.0, Math.PI);
  public static final ArmState PASS_TO_FRONT =
      ArmState.fromAbsolute(0.05, Math.PI / 2.0, Math.PI / 4.0);

  public static final ArmState OLD_FRONT_INTAKE = ArmState.fromAbsolute(0.454, -0.983, -0.055);

  public static final ArmState OLD_FRONT_SINGLE_SUBSTATION_CONE =
      ArmState.fromAbsolute(0.425006, 0.128855, -0.305);
  public static final ArmState OLD_FRONT_SINGLE_SUBSTATION_CUBE =
      ArmState.fromAbsolute(0.543571, -0.367516, 0.445646);
  public static final ArmState OLD_BACK_DOUBLE_SUBSTATION = ArmState.fromAbsolute(0, 2.8, Math.PI);

  public static final ArmState OLD_FRONT_MID_CONE =
      ArmState.fromAbsolute(0.061612, 0.493303, 0.001378);

  public static final ArmState OLD_BACK_MID_CONE = STOW; // TODO
  public static final ArmState OLD_BACK_HIGH_CONE = ArmState.fromAbsolute(0.253, 3.072, 2.5);
  // ele 0.2475
  public static final ArmState OLD_FRONT_MID_CUBE =
      ArmState.fromAbsolute(0.11362, 0.458149, 0.353288);
  public static final ArmState OLD_FRONT_HIGH_CUBE =
      ArmState.fromAbsolute(0.113502, 0.333258, 0.353208);

  public static final ArmState OLD_BACK_MID_CUBE = OLD_FRONT_MID_CUBE; // TODO
  public static final ArmState OLD_BACK_HIGH_CUBE = ArmState.fromAbsolute(0.245, 2.75, 3.17);

  public static final ArmState NEW_GROUND_CONE_INTAKE =
      ArmState.fromRelative(0.618, -0.714, -0.723);

  public static final ArmState NEW_GROUND_CUBE_INTAKE = ArmState.fromRelative(0, 0, 0);

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
    return Constants.ROBOT_TYPE == RobotType.WHIPLASH_ROLLER
        ? fromOperatorNew(height, gamePiece, side)
        : fromOperatorOld(height, gamePiece, side);
  }

  /** Returns a PlacementState from scoring parameters based on new arm design */
  public static ArmState fromOperatorNew(Level height, GamePiece gamePiece, Side side) {
    return switch (height) {
      case LOW -> switch (gamePiece) {
        case CONE -> NEW_GROUND_CONE_INTAKE;
        case CUBE -> NEW_GROUND_CUBE_INTAKE;
      };
      case MID -> switch (gamePiece) {
        case CONE -> side == Side.FRONT ? OLD_FRONT_MID_CONE : OLD_BACK_MID_CONE;
        case CUBE -> side == Side.FRONT ? OLD_FRONT_MID_CUBE : OLD_BACK_MID_CUBE;
      };
      case HIGH -> switch (gamePiece) {
        case CONE -> OLD_BACK_HIGH_CONE;
        case CUBE -> side == Side.FRONT ? OLD_FRONT_HIGH_CUBE : OLD_BACK_HIGH_CUBE;
      };
      case SINGLE_SUBSTATION -> switch (gamePiece) {
        case CONE -> OLD_FRONT_SINGLE_SUBSTATION_CONE;
        case CUBE -> OLD_FRONT_SINGLE_SUBSTATION_CUBE;
      };
      case DOUBLE_SUBSTATION -> OLD_BACK_DOUBLE_SUBSTATION;
    };
  }

  /** Returns a PlacementState from scoring parameters based on old arm design */
  public static ArmState fromOperatorOld(Level height, GamePiece gamePiece, Side side) {
    return switch (height) {
      case LOW -> OLD_FRONT_INTAKE;
      case MID -> switch (gamePiece) {
        case CONE -> side == Side.FRONT ? OLD_FRONT_MID_CONE : OLD_BACK_MID_CONE;
        case CUBE -> side == Side.FRONT ? OLD_FRONT_MID_CUBE : OLD_BACK_MID_CUBE;
      };
      case HIGH -> switch (gamePiece) {
        case CONE -> OLD_BACK_HIGH_CONE;
        case CUBE -> side == Side.FRONT ? OLD_FRONT_HIGH_CUBE : OLD_BACK_HIGH_CUBE;
      };
      case SINGLE_SUBSTATION -> switch (gamePiece) {
        case CONE -> OLD_FRONT_SINGLE_SUBSTATION_CONE;
        case CUBE -> OLD_FRONT_SINGLE_SUBSTATION_CUBE;
      };
      case DOUBLE_SUBSTATION -> OLD_BACK_DOUBLE_SUBSTATION;
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
}
