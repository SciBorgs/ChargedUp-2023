package org.sciborgs1155.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Optional;
import java.util.Set;
import java.util.function.DoubleUnaryOperator;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Constants.Dimensions;
import org.sciborgs1155.robot.Constants.Elbow;
import org.sciborgs1155.robot.Constants.Elevator;
import org.sciborgs1155.robot.Constants.RobotType;

/** ArmState class to store relative angles for the arm. */
public record ArmState(double elevatorHeight, Rotation2d elbowAngle, Rotation2d wristAngle) {

  // ARM STATES
  static final ArmState PASS_TO_BACK = fromAbsolute(0.03, Math.PI / 2.0, Math.PI);
  static final ArmState PASS_TO_FRONT = fromAbsolute(0.05, Math.PI / 2.0, Math.PI / 4.0);

  // OLD ARM SPECIFIC STATES
  static final ArmState OLD_INITIAL = fromRelative(Elevator.ZERO_OFFSET, Elbow.OFFSET, Math.PI);
  static final ArmState OLD_STOW = fromRelative(0, 1.218, Math.PI / 2.0);
  static final ArmState OLD_SAFE =
      fromAbsolute(Elevator.ZERO_OFFSET, Elbow.OFFSET + 0.1, Math.PI / 2);

  static final ArmState OLD_GROUND_INTAKE = fromAbsolute(0.454, -0.983, -0.055);
  static final ArmState OLD_SINGLESUB_CONE = fromAbsolute(0.425, 0.129, -0.305);
  static final ArmState OLD_SINGLESUB_CUBE = fromAbsolute(0.544, -0.368, 0.446);
  static final ArmState OLD_DOUBLESUB = fromAbsolute(0, 2.8, Math.PI);

  static final ArmState OLD_MID_CONE = fromAbsolute(0.062, 0.493, 0.001);
  static final ArmState OLD_MID_CUBE = fromAbsolute(0.114, 0.458, 0.353);
  static final ArmState OLD_HIGH_CONE = fromAbsolute(0.253, 3.072, 2.5);
  static final ArmState OLD_HIGH_CUBE = fromAbsolute(0.114, 0.333, 0.353);

  // NEW ARM SPECIFIC STATES
  static final ArmState NEW_INITIAL = fromRelative(Elevator.ZERO_OFFSET, Elbow.OFFSET, 2.4);
  static final ArmState NEW_STOW = fromEndpoint(0.425, 0.394, 1.147).get();

  static final ArmState NEW_GROUND_CONE = fromRelative(0.618, -0.714, -0.723);
  static final ArmState NEW_GROUND_CUBE = fromEndpoint(0.331, 0.225, -0.253).get();
  static final ArmState NEW_SINGLESUB_CONE = OLD_SINGLESUB_CONE;
  static final ArmState NEW_SINGLESUB_CUBE = OLD_SINGLESUB_CUBE;
  static final ArmState NEW_DOUBLESUB_CONE = NEW_STOW;
  static final ArmState NEW_DOUBLESUB_CUBE = NEW_STOW;

  static final ArmState NEW_MID_CONE = fromRelative(0.466, 0.24, -1.54);
  static final ArmState NEW_MID_CUBE = fromRelative(0.541, -0.34, 0.85);
  static final ArmState NEW_HIGH_CONE = fromRelative(0.366, 2.95, 0.02);
  static final ArmState NEW_HIGH_CUBE = fromRelative(0.344, 2.9, 1.4);

  static final Set<ArmState> OLD_PRESETS =
      Set.of(
          PASS_TO_BACK,
          PASS_TO_FRONT,
          OLD_INITIAL,
          OLD_STOW,
          OLD_SAFE,
          OLD_GROUND_INTAKE,
          OLD_SINGLESUB_CONE,
          OLD_SINGLESUB_CUBE,
          OLD_DOUBLESUB,
          OLD_MID_CONE,
          OLD_MID_CUBE,
          OLD_HIGH_CONE,
          OLD_HIGH_CUBE);
  static final Set<ArmState> NEW_PRESETS =
      Set.of(
          PASS_TO_BACK,
          PASS_TO_FRONT,
          NEW_INITIAL,
          NEW_STOW,
          NEW_GROUND_CONE,
          NEW_GROUND_CUBE,
          NEW_SINGLESUB_CONE,
          NEW_SINGLESUB_CUBE,
          // NEW_DOUBLESUB_CONE,
          // NEW_DOUBLESUB_CUBE,
          NEW_MID_CONE,
          NEW_MID_CUBE,
          NEW_HIGH_CONE,
          NEW_HIGH_CUBE);

  /** Represents the side of the robot the arm is on */
  public static enum Side {
    FRONT(0),
    BACK(Math.PI);

    public final double angle;

    private Side(double angle) {
      this.angle = angle;
    }
  }

  /** Represents the if the robot arm is facing up or down */
  public static enum End {
    TOP,
    BOTTOM,
  }

  /** Represents the game piece the robot is holding */
  public static enum GamePiece {
    CONE,
    CUBE,
  }

  /** Represents the part of the game the robot is interacting with */
  public static enum Goal {
    HIGH,
    MID,
    LOW,
    SINGLE_SUBSTATION,
    DOUBLE_SUBSTATION,
  }

  /**
   * Returns a new {@link ArmState} from angles in radians, normalized from -pi to pi, with the
   * wrist state relative to the chassis
   */
  public static ArmState fromAbsolute(double elevatorHeight, double elbowAngle, double wristAngle) {
    return fromRelative(elevatorHeight, elbowAngle, wristAngle - elbowAngle);
  }

  /**
   * Returns a new {@link ArmState} from angles in radians, normalized from -pi to pi, with the
   * wrist state relative to the forearm
   */
  public static ArmState fromRelative(double elevatorHeight, double elbowAngle, double wristAngle) {
    return new ArmState(
        elevatorHeight,
        Rotation2d.fromRadians(MathUtil.angleModulus(elbowAngle)),
        Rotation2d.fromRadians(MathUtil.angleModulus(wristAngle)));
  }

  /** Creates a PlacementState from an absolute 3d array */
  public static ArmState fromArray(double[] state) {
    return fromAbsolute(state[0], state[1], state[2]);
  }

  private static double round(double value) {
    return (double) Math.round(value * 1000000d) / 1000000d;
  }

  public ArmState {
    elevatorHeight = round(elevatorHeight);
    elbowAngle = Rotation2d.fromRadians(round(elbowAngle.getRadians()));
    wristAngle = Rotation2d.fromRadians(round(wristAngle.getRadians()));
  }

  /** Returns an array representation of the absolute state */
  public double[] toArray() {
    return new double[] {
      elevatorHeight, elbowAngle.getRadians(), wristAngle.getRadians() + elbowAngle.getRadians()
    };
  }

  /** The side of the robot the arm is on (front vs back) */
  public Side side() {
    return elbowAngle.getCos() > 0 ? Side.FRONT : Side.BACK;
  }

  /** The end of the robot the arm is on (top vs bottom) */
  public End end() {
    return elbowAngle.getSin() > 0 ? End.TOP : End.BOTTOM;
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
  public static ArmState fromGoal(Goal height, GamePiece gamePiece) {
    return Constants.ROBOT_TYPE == RobotType.WHIPLASH_ROLLER
        ? fromGoalNew(height, gamePiece)
        : fromGoalOld(height, gamePiece);
  }

  public static ArmState initial() {
    return Constants.ROBOT_TYPE == RobotType.WHIPLASH_ROLLER ? NEW_STOW : OLD_STOW;
  }

  public static ArmState stow() {
    return Constants.ROBOT_TYPE == RobotType.WHIPLASH_ROLLER ? NEW_INITIAL : OLD_INITIAL;
  }

  /** Returns a PlacementState from scoring parameters based on new arm design */
  public static ArmState fromGoalNew(Goal height, GamePiece gamePiece) {
    return switch (height) {
      case LOW -> switch (gamePiece) {
        case CONE -> NEW_GROUND_CONE;
        case CUBE -> NEW_GROUND_CUBE;
      };
      case MID -> switch (gamePiece) {
        case CONE -> NEW_MID_CONE;
        case CUBE -> NEW_MID_CUBE;
      };
      case HIGH -> switch (gamePiece) {
        case CONE -> NEW_HIGH_CONE;
        case CUBE -> NEW_HIGH_CUBE;
      };
      case SINGLE_SUBSTATION -> switch (gamePiece) {
        case CONE -> NEW_SINGLESUB_CONE;
        case CUBE -> NEW_SINGLESUB_CUBE;
      };
      case DOUBLE_SUBSTATION -> switch (gamePiece) {
        case CONE -> NEW_DOUBLESUB_CONE;
        case CUBE -> NEW_DOUBLESUB_CUBE;
      };
    };
  }

  /** Returns a PlacementState from scoring parameters based on old arm design */
  public static ArmState fromGoalOld(Goal height, GamePiece gamePiece) {
    return switch (height) {
      case LOW -> OLD_GROUND_INTAKE;
      case MID -> switch (gamePiece) {
        case CONE -> OLD_MID_CONE;
        case CUBE -> OLD_MID_CUBE;
      };
      case HIGH -> switch (gamePiece) {
        case CONE -> OLD_HIGH_CONE;
        case CUBE -> OLD_HIGH_CUBE;
      };
      case SINGLE_SUBSTATION -> switch (gamePiece) {
        case CONE -> OLD_SINGLESUB_CONE;
        case CUBE -> OLD_SINGLESUB_CUBE;
      };
      case DOUBLE_SUBSTATION -> OLD_DOUBLESUB;
    };
  }

  /**
   * Creates a new ArmState based on the provided pose for the endpoint using inverse kinematics.
   *
   * @param x The horizontal translation of the endpoint, in meters.
   * @param y The vertical translation of the endpoint, in meters.
   * @param theta The absolute angle of the endpoint from the horizontal, in radians.
   * @return An optional, either containing the computed ArmState or None if such a state is
   *     impossible.
   */
  public static Optional<ArmState> fromEndpoint(double x, double y, double theta) {
    return fromEndpoint(new Pose2d(new Translation2d(x, y), Rotation2d.fromRadians(theta)));
  }

  /**
   * Creates a new ArmState based on the provided pose for the endpoint using inverse kinematics.
   *
   * @param endpoint A Pose2d, where translation represents the coordinates of the wrist joint and
   *     rotation represents the rotation of the wrist.
   * @return An optional, either containing the computed ArmState or None if such a state is
   *     impossible.
   */
  public static Optional<ArmState> fromEndpoint(Pose2d endpoint) {
    // find coordinate of wrist (end of forearm) relative to the elevator on top of our chassis
    var translation = endpoint.getTranslation().minus(Dimensions.BASE);

    // cos(theta) = adj / hypot
    double elbowAngle = Math.acos(translation.getX() / Dimensions.FOREARM_LENGTH);

    // fn to calculate needed elevator height from the difference between our goal and the y
    // component of our forearm
    DoubleUnaryOperator elevatorHeight =
        theta -> translation.getY() - Math.sin(theta) * Dimensions.FOREARM_LENGTH;

    // try again with the other possible elbow angle, in case we have an invalid state
    if (elevatorHeight.applyAsDouble(elbowAngle) < Elevator.MIN_HEIGHT
        || elevatorHeight.applyAsDouble(elbowAngle) > Elevator.MAX_HEIGHT) {
      elbowAngle *= -1; // reflect elbow over x-axis
    }

    // finalize elevator height
    double finalElevatorHeight = elevatorHeight.applyAsDouble(elbowAngle);

    // discard if the state is still invalid
    if (finalElevatorHeight < Elevator.MIN_HEIGHT
        || finalElevatorHeight > Elevator.MAX_HEIGHT
        || Double.isNaN(elbowAngle)) {
      return Optional.empty();
    }

    return Optional.of(
        fromAbsolute(finalElevatorHeight, elbowAngle, endpoint.getRotation().getRadians()));
  }

  /** Performs forward kinematics to return the claw's position as a {@link Pose2d} */
  public Pose2d getEndpoint() {
    return new Pose2d(
        Dimensions.BASE
            .plus(new Translation2d(0, elevatorHeight))
            .plus(new Translation2d(Dimensions.FOREARM_LENGTH, elbowAngle)),
        elbowAngle.plus(wristAngle));
  }

  public static ArmState nextState(ArmState state, Transform2d change) {
    Optional<ArmState> newState = fromEndpoint(state.getEndpoint().plus(change));
    return newState.isPresent() ? newState.get() : state;
  }
}
