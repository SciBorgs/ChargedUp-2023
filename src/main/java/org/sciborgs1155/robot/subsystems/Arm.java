package org.sciborgs1155.robot.subsystems;

import static org.sciborgs1155.robot.Constants.Arm.*;
import static org.sciborgs1155.robot.Ports.Arm.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import org.sciborgs1155.lib.Derivative;
import org.sciborgs1155.lib.Trajectory;
import org.sciborgs1155.lib.Trajectory.State;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Constants.Dimensions;
import org.sciborgs1155.robot.Robot;
import org.sciborgs1155.robot.util.Visualizer;

public class Arm extends SubsystemBase implements Loggable, AutoCloseable {

  @Log(name = "elbow applied output", methodName = "getAppliedOutput")
  private final CANSparkMax elbow = Elbow.MOTOR.build(MotorType.kBrushless, MIDDLE_ELBOW_MOTOR);

  private final CANSparkMax elbowLeft = Elbow.MOTOR.build(MotorType.kBrushless, LEFT_ELBOW_MOTOR);
  private final CANSparkMax elbowRight = Elbow.MOTOR.build(MotorType.kBrushless, RIGHT_ELBOW_MOTOR);

  @Log(name = "wrist applied output", methodName = "getAppliedOutput")
  private final CANSparkMax wrist = Wrist.MOTOR.build(MotorType.kBrushless, WRIST_MOTOR);

  @Log private final Encoder elbowEncoder = new Encoder(ELBOW_ENCODER[0], ELBOW_ENCODER[1]);
  private final EncoderSim elbowEncoderSim = new EncoderSim(elbowEncoder);

  @Log(name = "wrist velocity", methodName = "getVelocity")
  private final AbsoluteEncoder wristEncoder = wrist.getAbsoluteEncoder(Type.kDutyCycle);

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
          1 / Elbow.CONVERSION.gearing(),
          SingleJointedArmSim.estimateMOI(Dimensions.FOREARM_LENGTH, Dimensions.FOREARM_MASS),
          Dimensions.FOREARM_LENGTH,
          Dimensions.ELBOW_MIN_ANGLE,
          Dimensions.ELBOW_MAX_ANGLE,
          true);
  private final SingleJointedArmSim wristSim =
      new SingleJointedArmSim(
          DCMotor.getNEO(1),
          1,
          SingleJointedArmSim.estimateMOI(Dimensions.CLAW_LENGTH, Dimensions.CLAW_MASS),
          Dimensions.CLAW_LENGTH,
          Dimensions.WRIST_MIN_ANGLE,
          Dimensions.WRIST_MAX_ANGLE,
          false);

  private final Visualizer positionVisualizer;
  private final Visualizer setpointVisualizer;

  public Arm(Visualizer positionVisualizer, Visualizer setpointVisualizer) {
    elbowLeft.follow(elbow);
    elbowRight.follow(elbow);

    elbowEncoder.setDistancePerPulse(Elbow.CONVERSION.factor());
    wristEncoder.setPositionConversionFactor(Wrist.CONVERSION.factor());
    wristEncoder.setVelocityConversionFactor(Wrist.CONVERSION.factor() / 60.0);

    // set wrist duty cycle absolute encoder frame periods to be the same as our tickrate
    // periodic frames 3 and 4 are useless to us, so to improve performance we set them to 1155 ms
    wrist.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1155);
    wrist.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1155);
    wrist.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
    wrist.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);

    elbow.burnFlash();
    elbowLeft.burnFlash();
    elbowRight.burnFlash();
    wrist.burnFlash();

    elbowSetpoint = new State(getElbowPosition().getRadians(), 0, 0);
    wristSetpoint = new State(getRelativeWristPosition().getRadians(), 0, 0);

    this.positionVisualizer = positionVisualizer;
    this.setpointVisualizer = setpointVisualizer;
  }

  /** Elbow position relative to the chassis */
  @Log(name = "elbow position", methodName = "getDegrees")
  public Rotation2d getElbowPosition() {
    return Rotation2d.fromRadians(elbowEncoder.getDistance() + Elbow.ELBOW_OFFSET);
  }

  /** Wrist position relative to the forearm */
  @Log(name = "relative wrist position", methodName = "getDegrees")
  public Rotation2d getRelativeWristPosition() {
    // encoder is zeroed fully folded in, which is actually PI, so we offset by -PI
    return Rotation2d.fromRadians(
        Robot.isReal() ? wristEncoder.getPosition() - Math.PI : wristSim.getAngleRads());
  }

  /** Wrist position relative to chassis */
  @Log(name = "absolute wrist position", methodName = "getDegrees")
  public Rotation2d getAbsoluteWristPosition() {
    return getRelativeWristPosition().plus(getElbowPosition());
  }

  /** Sets the elbows's setpoint, in radians */
  public void setElbowSetpoint(State state) {
    elbowSetpoint = state;
  }

  /** Sets the wrist's setpoint, in radians */
  public void setWristSetpoint(State state) {
    wristSetpoint = state;
  }

  /** Follows a {@link TrapezoidProfile} for each joint's relative position */
  public Command followProfile(Rotation2d elbowPosition, Rotation2d wristPosition) {
    var elbowGoal = new TrapezoidProfile.State(elbowPosition.getRadians(), 0);
    var wristGoal = new TrapezoidProfile.State(wristPosition.getRadians(), 0);
    var elbowAccel = new Derivative();
    var wristAccel = new Derivative();

    return runOnce(
            () -> {
              elbowAccel.reset();
              wristAccel.reset();
            })
        .andThen(
            run(
                () -> {
                  var elbowProfile =
                      new TrapezoidProfile(
                          Elbow.CONSTRAINTS, elbowGoal, elbowSetpoint.trapezoidState());
                  var wristProfile =
                      new TrapezoidProfile(
                          Wrist.CONSTRAINTS, wristGoal, wristSetpoint.trapezoidState());

                  var elbowSetpoint = elbowProfile.calculate(elbowFeedback.getPeriod());
                  var wristSetpoint = wristProfile.calculate(wristFeedback.getPeriod());

                  setElbowSetpoint(
                      new State(
                          elbowSetpoint.position,
                          elbowSetpoint.velocity,
                          elbowAccel.calculate(elbowSetpoint.velocity)));
                  setWristSetpoint(
                      new State(
                          wristSetpoint.position,
                          wristSetpoint.velocity,
                          wristAccel.calculate(wristSetpoint.velocity)));
                }))
        .until(
            () ->
                elbowFeedback.atSetpoint()
                    && elbowGoal.equals(elbowSetpoint.trapezoidState())
                    && wristFeedback.atSetpoint()
                    && wristGoal.equals(wristSetpoint.trapezoidState()));
  }

  /** Follows a {@link Trajectory} for each joint's relative position */
  // TODO check if relative or absolute trajectories are loaded
  public Command followTrajectory(Trajectory elbowTrajectory, Trajectory wristTrajectory) {
    DriverStation.reportError(
        "SUPPLIED ELBOW AND WRIST TRAJECTORIES DO NOT HAVE EQUAL TOTAL TIMES", false);

    Timer timer = new Timer();
    return runOnce(timer::start)
        .andThen(
            run(
                () -> {
                  setElbowSetpoint(elbowTrajectory.sample(timer.get()));
                  setWristSetpoint(wristTrajectory.sample(timer.get()));
                }))
        .until(() -> timer.hasElapsed(elbowTrajectory.getTotalTime()));
  }

  @Override
  public void periodic() {
    double elbowFB =
        elbowFeedback.calculate(getElbowPosition().getRadians(), elbowSetpoint.position());
    double elbowFF =
        elbowFeedforward.calculate(
            elbowSetpoint.position(), elbowSetpoint.velocity(), elbowSetpoint.acceleration());
    elbow.setVoltage(elbowFB + elbowFF);

    // wrist feedback is calculated using an absolute angle setpoint, rather than a relative one
    // this means the extra voltage calculated to cancel out gravity is kG * cos(θ + ϕ), where θ is
    // the elbow setpoint and ϕ is the wrist setpoint
    // the elbow angle is used as a setpoint instead of current position because we're using a
    // profiled pid controller, which means setpoints are achievable states, rather than goals
    double wristFB =
        wristFeedback.calculate(getRelativeWristPosition().getRadians(), wristSetpoint.position());
    double wristFF =
        wristFeedforward.calculate(
            wristSetpoint.position() + elbowSetpoint.position(),
            wristSetpoint.velocity(),
            wristSetpoint.acceleration());
    wrist.setVoltage(wristFB + wristFF);

    positionVisualizer.setArmAngles(getElbowPosition(), getRelativeWristPosition());
    setpointVisualizer.setArmAngles(
        Rotation2d.fromRadians(elbowFeedback.getSetpoint()),
        Rotation2d.fromRadians(wristFeedback.getSetpoint()));
  }

  @Override
  public void simulationPeriodic() {
    elbowSim.setInputVoltage(elbow.getAppliedOutput());
    elbowSim.update(Constants.RATE);
    elbowEncoderSim.setDistance(elbowSim.getAngleRads());
    elbowEncoderSim.setRate(elbowSim.getVelocityRadPerSec());

    wristSim.setInputVoltage(wrist.getAppliedOutput());
    wristSim.update(Constants.RATE);
  }

  @Override
  public void close() {
    elbow.close();
    elbowLeft.close();
    elbowRight.close();
    wrist.close();
  }
}
