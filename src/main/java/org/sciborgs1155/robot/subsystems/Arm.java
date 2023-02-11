package org.sciborgs1155.robot.subsystems;

import static org.sciborgs1155.robot.Constants.Arm.*;
import static org.sciborgs1155.robot.Ports.Arm.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import org.sciborgs1155.lib.Derivative;
import org.sciborgs1155.lib.Visualizer;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Constants.Dimensions;
import org.sciborgs1155.robot.Constants.Motors;

public class Arm extends SubsystemBase implements Loggable, AutoCloseable {

  /** Represents the side of the robot the arm is on */
  public enum Side {
    FRONT,
    BACK;
  }

  /** State class to store relative angles for the arm. */
  public record State(Rotation2d elbowAngle, Rotation2d wristAngle) implements Sendable {

    /**
     * Returns a new {@link State} from angles in radians, with the wrist state relative to the
     * chassis
     */
    public static State fromAbsolute(double elbowAngle, double wristAngle) {
      return new State(
          Rotation2d.fromRadians(elbowAngle), Rotation2d.fromRadians(wristAngle - elbowAngle));
    }

    /**
     * Returns a new {@link State} from angles in radians, with the wrist state relative to the
     * forearm
     */
    public static State fromRelative(double elbowAngle, double wristAngle) {
      return new State(Rotation2d.fromRadians(elbowAngle), Rotation2d.fromRadians(wristAngle));
    }

    /** The side of the robot the arm is on */
    public Side toSide() {
      return elbowAngle.getCos() > 0 ? Side.FRONT : Side.BACK;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
      builder.addDoubleProperty("elbow angle", () -> elbowAngle.getDegrees(), null);
      builder.addDoubleProperty("relative wrist angle", () -> wristAngle.getDegrees(), null);
      builder.addDoubleProperty(
          "absolute wrist angle", () -> elbowAngle.plus(wristAngle).getDegrees(), null);
    }
  }

  @Log(name = "wrist applied output", methodName = "getAppliedOutput")
  private final CANSparkMax wrist = Motors.WRIST.build(MotorType.kBrushless, WRIST_MOTOR);

  @Log(name = "elbow applied output", methodName = "getAppliedOutput")
  private final CANSparkMax elbowLead = Motors.ELBOW.build(MotorType.kBrushless, LEFT_ELBOW_MOTOR);

  private final CANSparkMax elbowFollow =
      Motors.ELBOW.build(MotorType.kBrushless, RIGHT_ELBOW_MOTOR);

  @Log(name = "wrist velocity", methodName = "getVelocity")
  private final RelativeEncoder wristEncoder = wrist.getEncoder();

  @Log(name = "elbow velocity", methodName = "getVelocity")
  private final RelativeEncoder elbowEncoder = elbowLead.getEncoder();

  @Log
  private final ProfiledPIDController wristFeedback =
      new ProfiledPIDController(Wrist.kP, Wrist.kI, Wrist.kD, Wrist.CONSTRAINTS);

  @Log
  private final ProfiledPIDController elbowFeedback =
      new ProfiledPIDController(Elbow.kP, Elbow.kI, Elbow.kD, Elbow.CONSTRAINTS);

  private final ArmFeedforward wristFeedforward =
      new ArmFeedforward(Wrist.kS, Wrist.kG, Wrist.kV, Wrist.kA);
  private final ArmFeedforward elbowFeedforward =
      new ArmFeedforward(Elbow.kS, Elbow.kG, Elbow.kV, Elbow.kA);

  @Log(name = "wrist acceleration", methodName = "getLastOutput")
  private final Derivative wristAccel = new Derivative();

  @Log(name = "elbow acceleration", methodName = "getLastOutput")
  private final Derivative elbowAccel = new Derivative();

  private final SingleJointedArmSim elbowSim =
      new SingleJointedArmSim(
          DCMotor.getNEO(2),
          5,
          SingleJointedArmSim.estimateMOI(Dimensions.FOREARM_LENGTH, 2),
          Dimensions.FOREARM_LENGTH,
          Dimensions.ELBOW_MIN_ANGLE,
          Dimensions.ELBOW_MAX_ANGLE,
          true);
  private final SingleJointedArmSim wristSim =
      new SingleJointedArmSim(
          DCMotor.getNeo550(1),
          5,
          SingleJointedArmSim.estimateMOI(Dimensions.CLAW_LENGTH, 1),
          Dimensions.CLAW_LENGTH,
          Dimensions.WRIST_MIN_ANGLE,
          Dimensions.WRIST_MAX_ANGLE,
          true);

  public Arm() {
    elbowFollow.follow(elbowLead);

    elbowEncoder.setPositionConversionFactor(Elbow.GEAR_RATIO * Elbow.MOVEMENT_PER_SPIN);
    elbowEncoder.setVelocityConversionFactor(Elbow.GEAR_RATIO);
  }

  /** Relative state of the arm */
  @Log(name = "state")
  public State getState() {
    return State.fromRelative(elbowEncoder.getPosition(), wristEncoder.getPosition());
  }

  /** Relative goal of the arm */
  @Log(name = "goal")
  public State getGoal() {
    return State.fromRelative(elbowFeedback.getGoal().position, wristFeedback.getGoal().position);
  }

  /** Elbow is at goal */
  @Log(name = "elbow at goal")
  public boolean atElbowGoal() {
    return elbowFeedback.atGoal();
  }

  /** Wrist is at goal */
  @Log(name = "wrist at goal")
  public boolean atWristGoal() {
    return wristFeedback.atGoal();
  }

  /** Wrist and elbow are at goal */
  public boolean atGoal() {
    return atElbowGoal() && atWristGoal();
  }

  /** Sets arm and wrist goals, with the wrist goal relative to the arm */
  public Command setGoal(State goal) {
    return runOnce(() -> elbowFeedback.setGoal(goal.elbowAngle.getRadians()))
        .andThen(() -> wristFeedback.setGoal(goal.wristAngle.getRadians()));
  }

  public Command runToGoal(State goal) {
    return setGoal(goal).andThen(Commands.waitUntil(this::atGoal));
  }

  /**
   * Returns the corresponding arm state given a target Math can be found <a
   * href="https://robotacademy.net.au/lesson/inverse-kinematics-for-a-2-joint-robot-arm-using-geometry">here.</a>
   */
  public State inverseRR(Transform3d target) {
    double wristAngle =
        -Math.acos(
            (Math.pow(target.getY(), 2)
                    + Math.pow(target.getZ(), 2)
                    - Math.pow(Dimensions.FOREARM_LENGTH, 2)
                    - Math.pow(Dimensions.CLAW_LENGTH, 2))
                / 2
                * Dimensions.CLAW_LENGTH
                * Dimensions.FOREARM_LENGTH);
    double elbowAngle =
        Math.atan2(target.getZ(), target.getY())
            + Math.atan2(
                Dimensions.CLAW_LENGTH * Math.sin(wristAngle),
                Dimensions.FOREARM_LENGTH + Dimensions.CLAW_LENGTH * Math.cos(wristAngle));
    return new State(Rotation2d.fromRadians(elbowAngle), Rotation2d.fromRadians(wristAngle));
  }

  @Override
  public void periodic() {
    double elbowfb = elbowFeedback.calculate(elbowEncoder.getPosition());
    double elbowff =
        elbowFeedforward.calculate(
            elbowFeedback.getSetpoint().position,
            elbowFeedback.getSetpoint().velocity,
            elbowAccel.calculate(elbowFeedback.getSetpoint().velocity));
    elbowLead.setVoltage(elbowfb + elbowff);

    // wrist feedback is calculated using an absolute angle setpoint, rather than a
    // relative one
    // this means the extra voltage calculated to cancel out gravity is kG * cos(θ +
    // ϕ), where θ is
    // the elbow setpoint and ϕ is the wrist setpoint
    // the elbow angle is used as a setpoint instead of current position because
    // we're using a
    // profiled pid controller, which means setpoints are achievable states, rather
    // than goals
    double wristfb = wristFeedback.calculate(wristEncoder.getPosition());
    double wristff =
        wristFeedforward.calculate(
            wristFeedback.getSetpoint().position + elbowFeedback.getSetpoint().position,
            wristFeedback.getSetpoint().velocity,
            wristAccel.calculate(wristFeedback.getSetpoint().velocity));
    wrist.setVoltage(wristfb + wristff);

    Visualizer.getInstance().setArmPositions(getState());
  }

  @Override
  public void simulationPeriodic() {
    elbowSim.setInputVoltage(elbowLead.getAppliedOutput());
    elbowSim.update(Constants.RATE);
    elbowEncoder.setPosition(elbowSim.getAngleRads());

    wristSim.setInputVoltage(wrist.getAppliedOutput());
    wristSim.update(Constants.RATE);
    wristEncoder.setPosition(wristSim.getAngleRads());
  }

  @Override
  public void close() {
    wrist.close();
    elbowLead.close();
    elbowFollow.close();
  }
}
