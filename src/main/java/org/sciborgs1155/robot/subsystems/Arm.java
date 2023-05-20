package org.sciborgs1155.robot.subsystems;

import static org.sciborgs1155.robot.Constants.Arm.*;
import static org.sciborgs1155.robot.Ports.Arm.*;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import org.sciborgs1155.lib.DeferredCommand;
import org.sciborgs1155.lib.Derivative;
import org.sciborgs1155.lib.Trajectory;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Constants.Dimensions;
import org.sciborgs1155.robot.Robot;
import org.sciborgs1155.robot.subsystems.arm.JointIO;
import org.sciborgs1155.robot.subsystems.arm.JointSim;
import org.sciborgs1155.robot.subsystems.arm.MAXWristJoint;
import org.sciborgs1155.robot.util.Visualizer;

public class Arm extends SubsystemBase implements Loggable, AutoCloseable {

  public static record JointConfig(
      DCMotor gearbox,
      double gearing,
      double length,
      double mass,
      double minAngle,
      double maxAngle) {}

  public interface Joint extends Sendable, AutoCloseable {
    public Rotation2d getRotation();

    public State getState();

    public State getDesiredState();

    public void updateDesiredState(State state);

    @Override
    default void initSendable(SendableBuilder builder) {
      // TODO Auto-generated method stub
    }
  }

  public interface Elevator extends Sendable, AutoCloseable {
    public double getHeight();

    public State getState();

    public State getDesiredState();

    public void updateDesiredState(State state);

    public boolean isFailing();

    @Override
    default void initSendable(SendableBuilder builder) {
      // TODO Auto-generated method stub
    }
  }

  @Log(name = "elbow applied output", methodName = "getAppliedOutput")
  private final CANSparkMax elbow = Elbow.MOTOR.build(MotorType.kBrushless, MIDDLE_ELBOW_MOTOR);

  private final CANSparkMax elbowLeft = Elbow.MOTOR.build(MotorType.kBrushless, LEFT_ELBOW_MOTOR);
  private final CANSparkMax elbowRight = Elbow.MOTOR.build(MotorType.kBrushless, RIGHT_ELBOW_MOTOR);
  private boolean working = true;

  private final JointIO elbow;
  private final JointIO elbowLeft;
  private final JointIO elbowRight;

  @Log(name = "wrist applied output", methodName = "getAppliedOutput")
  private final JointIO wrist;

  private final Encoder elbowEncoder = new Encoder(ELBOW_ENCODER[0], ELBOW_ENCODER[1]);
  private final EncoderSim elbowEncoderSim = new EncoderSim(elbowEncoder);

  @Log(name = "wrist velocity", methodName = "getVelocity")
  private final AbsoluteEncoder wristEncoder;

  private final ArmFeedforward elbowFeedforward =
      new ArmFeedforward(Elbow.FF.s(), Elbow.FF.g(), Elbow.FF.v(), Elbow.FF.a());
  private final ArmFeedforward wristFeedforward =
      new ArmFeedforward(Wrist.FF.s(), Wrist.FF.g(), Wrist.FF.v(), Wrist.FF.a());

  @Log(name = "elbow feedback")
  @Log(name = "elbow at setpoint", methodName = "atSetpoint")
  private final PIDController elbowFeedback =
      new PIDController(Elbow.PID.p(), Elbow.PID.i(), Elbow.PID.d());

  @Log(name = "wrist feedback")
  @Log(name = "wrist at setpoint", methodName = "atSetpoint")
  private final PIDController wristFeedback =
      new PIDController(Wrist.PID.p(), Wrist.PID.i(), Wrist.PID.d());

  @Log(name = "elbow position setpoint", methodName = "position")
  @Log(name = "elbow velocity setpoint", methodName = "velocity")
  @Log(name = "elbow acceleration setpoint", methodName = "acceleration")
  private State elbowSetpoint;

  @Log(name = "wrist position setpoint", methodName = "position")
  @Log(name = "wrist velocity setpoint", methodName = "velocity")
  @Log(name = "wrist acceleration setpoint", methodName = "acceleration")
  private State wristSetpoint;

  private final SingleJointedArmSim elbowSim =
      new SingleJointedArmSim(
          DCMotor.getNEO(3),
          Elbow.GEARING,
          SingleJointedArmSim.estimateMOI(Dimensions.FOREARM_LENGTH, Dimensions.FOREARM_MASS),
          Dimensions.FOREARM_LENGTH,
          Elbow.MIN_ANGLE,
          Elbow.MAX_ANGLE,
          true);

  private final SingleJointedArmSim wristSim =
      new SingleJointedArmSim(
          DCMotor.getNEO(1),
          Wrist.GEARING,
          SingleJointedArmSim.estimateMOI(Dimensions.CLAW_LENGTH, Dimensions.CLAW_MASS),
          Dimensions.CLAW_LENGTH,
          Wrist.MIN_ANGLE,
          Wrist.MAX_ANGLE,
          false);

  private final Visualizer positionVisualizer;
  private final Visualizer setpointVisualizer;

  @Log private boolean stopped = false;
  // 'tis but a scratch (see: match 42) (the elbow is unplugged)
  @Log private boolean butAScratch = false;
  @Log private boolean wristLimp = false;

  public Arm(Visualizer positionVisualizer, Visualizer setpointVisualizer, boolean working) {
    // elbow = JointIO.create(MIDDLE_ELBOW_MOTOR, working, newWrist);
    // elbowLeft = JointIO.create(LEFT_ELBOW_MOTOR, working, newWrist);
    // elbowRight = JointIO.create(RIGHT_ELBOW_MOTOR, working, newWrist);
    if (working) {
      elbow = new ElbowIO(newWrist, MIDDLE_ELBOW_MOTOR);
      elbowLeft = new ElbowIO(newWrist, LEFT_ELBOW_MOTOR);
      elbowRight = new ElbowIO(newWrist, RIGHT_ELBOW_MOTOR);
      wrist = new MAXWristJoint(newWrist, WRIST_MOTOR);
    } else if (Robot.isReal()) {
      elbow = new JointSim();
      elbowLeft = new JointSim(newWrist);
      elbowRight = new JointSim();
      wrist = new JointSim();
    }

    elbowLeft.follow(elbow);
    elbowRight.follow(elbow);

    // elbowEncoder.setDistancePerPulse(Elbow.CONVERSION.factor());
    // wristEncoder.setPositionConversionFactor(Wrist.CONVERSION.factor());
    // wristEncoder.setVelocityConversionFactor(Wrist.CONVERSION.factor() / 60.0);

    // set wrist duty cycle absolute encoder frame periods to be the same as our tickrate
    // periodic frames 3 and 4 are useless to us, so to improve performance we set them to 1155 ms
    // wrist.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1155);
    // wrist.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1155);
    // wrist.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
    // wrist.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);

    // elbow.burnFlash();
    // elbowLeft.burnFlash();
    // elbowRight.burnFlash();
    // wrist.burnFlash();

    // elbowSetpoint = new State(getElbowPosition().getRadians(), 0, 0);
    // wristSetpoint = new State(Math.PI, 0, 0);

    this.positionVisualizer = positionVisualizer;
    this.setpointVisualizer = setpointVisualizer;

    // wristFeedback.setTolerance(0.3);
    // elbowFeedback.setTolerance(0.3);
  }

  // /** Elbow position relative to the chassis */
  @Log(name = "elbow position", methodName = "getRadians")
  public Rotation2d getElbowPosition() {
    return Rotation2d.fromRadians(elbowEncoder.getDistance() + Elbow.OFFSET);
  }

  // /** Wrist position relative to the forearm */
  @Log(name = "relative wrist position", methodName = "getRadians")
  public Rotation2d getRelativeWristPosition() {
    // encoder is zeroed fully folded in, which is actually PI, so we offset by -PI
    return Rotation2d.fromRadians(
        Robot.isReal() ? wristEncoder.getPosition() - Math.PI : wristSim.getAngleRads());
  }

  /** Wrist position relative to chassis */
  @Log(name = "absolute wrist position", methodName = "getRadians")
  public Rotation2d getAbsoluteWristPosition() {
    return getRelativeWristPosition().plus(getElbowPosition());
  }

  /**
   * If elbow is far enough from setpoint (we always use trapezoid profiling or trajectories), it is
   * dangerous. This method returns a debounced trigger for when the elbow encoder is likely
   * failing/not plugged in.
   */
  public Trigger onElbowFailing() {
    return new Trigger(() -> butAScratch).debounce(Elbow.FAILING_DEBOUNCE_TIME, DebounceType.kBoth);
  }

  /**
   * If wrist is not working, we cannot pass over. The wrist not working is determined in {@link
   * #periodic()}
   */
  @Log
  public boolean allowPassOver() {
    return !wristLimp;
  }

  /** Sets the position setpoints for the elbow and wrist, in radians */
  public Command setSetpoints(Rotation2d elbowAngle, Rotation2d wristAngle) {
    return runOnce(
        () -> {
          elbowSetpoint = new State(elbowAngle.getRadians(), 0, 0);
          wristSetpoint = new State(wristAngle.getRadians(), 0, 0);
        });
  }

  /** Returns the elbow setpoint as a {@link State} */
  public State getElbowSetpoint() {
    return elbowSetpoint;
  }

  /** Returns the wrist setpoint as a {@link State} */
  public State getWristSetpoint() {
    return wristSetpoint;
  }

  private TrapezoidProfile createElbowProfile(Rotation2d goal) {
    return new TrapezoidProfile(
        Elbow.CONSTRAINTS,
        new TrapezoidProfile.State(goal.getRadians(), 0),
        elbowSetpoint.trapezoidState());
  }

  private TrapezoidProfile createWristProfile(Rotation2d goal) {
    return new TrapezoidProfile(
        Wrist.CONSTRAINTS,
        new TrapezoidProfile.State(goal.getRadians(), 0),
        wristSetpoint.trapezoidState());
  }

  /** Follows a {@link TrapezoidProfile} for each joint's relative position */
  public Command followProfile(Rotation2d elbowGoal, Rotation2d wristGoal) {
    var elbowAccel = new Derivative();
    var wristAccel = new Derivative();

    return runOnce(
            () -> {
              elbowAccel.reset();
              wristAccel.reset();
            })
        .andThen(
            new DeferredCommand(
                () ->
                    new TrapezoidProfileCommand(
                            createElbowProfile(elbowGoal),
                            state ->
                                elbowSetpoint =
                                    new State(
                                        state.position,
                                        state.velocity,
                                        elbowAccel.calculate(state.velocity)))
                        .alongWith(
                            new TrapezoidProfileCommand(
                                createWristProfile(wristGoal),
                                state ->
                                    wristSetpoint =
                                        new State(
                                            state.position,
                                            state.velocity,
                                            wristAccel.calculate(state.velocity))))))
        .withName("following profile");
  }

  /** Follows a {@link Trajectory} for each joint's relative position */
  public Command followTrajectory(Trajectory elbowTrajectory, Trajectory wristTrajectory) {
    if (elbowTrajectory.totalTime() != wristTrajectory.totalTime()) {
      DriverStation.reportError(
          "SUPPLIED ELBOW AND WRIST TRAJECTORIES DO NOT HAVE EQUAL TOTAL TIMES", false);
    }

    return elbowTrajectory
        .follow(state -> elbowSetpoint = state)
        .alongWith(wristTrajectory.follow(state -> wristSetpoint = state, this))
        .withName("following trajectory");
  }

  public Command setStopped(boolean stopped) {
    return runOnce(() -> this.stopped = stopped);
  }

  @Override
  public void periodic() {
    double elbowFB =
        elbowFeedback.calculate(getElbowPosition().getRadians(), elbowSetpoint.position());
    double elbowFF =
        elbowFeedforward.calculate(
            elbowSetpoint.position(), elbowSetpoint.velocity(), elbowSetpoint.acceleration());

    elbow.setVoltage(stopped ? 0 : elbowFB + elbowFF);

    // wrist feedback is calculated using an absolute angle setpoint, rather than a relative one
    // this means the extra voltage calculated to cancel out gravity is kG * cos(θ + ϕ), where θ is
    // the elbow setpoint and ϕ is the wrist setpoint
    // the elbow angle is used as a setpoint instead of current position because we're using
    // trajectories, which means setpoints are achievable states, rather than goals
    double wristFB =
        wristFeedback.calculate(getRelativeWristPosition().getRadians(), wristSetpoint.position());
    double wristFF =
        wristFeedforward.calculate(
            wristSetpoint.position() + elbowSetpoint.position(),
            wristSetpoint.velocity(),
            wristSetpoint.acceleration());

    wrist.setVoltage(stopped || wristLimp ? 0 : wristFB + wristFF);

    positionVisualizer.setArmAngles(getElbowPosition(), getRelativeWristPosition());
    setpointVisualizer.setArmAngles(
        Rotation2d.fromRadians(elbowFeedback.getSetpoint()),
        Rotation2d.fromRadians(wristFeedback.getSetpoint()));

    // SAFETY CHECKS
    wristLimp =
        wristEncoder.getPosition() == 0
            && wristEncoder.getVelocity() == 0
            && wristSetpoint.position() != 0
            && Robot.isReal();
    butAScratch =
        elbowEncoder.getDistance() == 0 // no position reading
            && elbowEncoder.getRate() == 0 // no velocity reading
            && elbowSetpoint.position() != Elbow.OFFSET // elbow is not going to 0
            && elbow.getAppliedOutput() != 0; // elbow is trying to move
  }

  @Override
  public void simulationPeriodic() {
    elbowSim.setInputVoltage(elbow.getAppliedOutput());
    elbowSim.update(Constants.PERIOD);
    elbowEncoderSim.setDistance(elbowSim.getAngleRads() - Elbow.OFFSET);
    elbowEncoderSim.setRate(elbowSim.getVelocityRadPerSec());

    wristSim.setInputVoltage(wrist.getAppliedOutput());
    wristSim.update(Constants.PERIOD);
  }

  @Override
  public void close() {
    elbow.close();
    elbowLeft.close();
    elbowRight.close();
    wrist.close();
    elbowEncoder.close();
  }
}
