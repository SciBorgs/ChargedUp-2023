package org.sciborgs1155.robot.subsystems;

import static org.sciborgs1155.robot.Constants.Elevator.*;
import static org.sciborgs1155.robot.Ports.Elevator.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import org.sciborgs1155.lib.DeferredCommand;
import org.sciborgs1155.lib.Derivative;
import org.sciborgs1155.lib.TestableSubsystem;
import org.sciborgs1155.lib.Trajectory;
import org.sciborgs1155.lib.Trajectory.State;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Constants.Dimensions;
import org.sciborgs1155.robot.Robot;
import org.sciborgs1155.robot.util.Visualizer;

public class Elevator extends TestableSubsystem implements Loggable {

  @Log(name = "applied output", methodName = "getAppliedOutput")
  private CANSparkMax lead = MOTOR.build(MotorType.kBrushless, RIGHT_MOTOR);

  private CANSparkMax left = MOTOR.build(MotorType.kBrushless, LEFT_MOTOR);
  private CANSparkMax right = MOTOR.build(MotorType.kBrushless, MIDDLE_MOTOR);

  @Log private final Encoder encoder = new Encoder(ENCODER[0], ENCODER[1], true);
  private final EncoderSim simEncoder = new EncoderSim(encoder);

  // private final AbsoluteEncoder offsetEncoder = right.getAbsoluteEncoder(Type.kDutyCycle);

  private final ElevatorFeedforward ff = new ElevatorFeedforward(FF.s(), FF.g(), FF.v(), FF.a());

  private DigitalInput bottomSwitch = new DigitalInput(LIMIT_SWITCH);

  @Log
  @Log(name = "at setpoint", methodName = "atSetpoint")
  private final PIDController pid = new PIDController(PID.p(), PID.i(), PID.d());

  @Log(name = "position setpoint", methodName = "position")
  @Log(name = "velocity setpoint", methodName = "velocity")
  @Log(name = "acceleration setpoint", methodName = "acceleration")
  private State setpoint;

  private final LinearFilter current = LinearFilter.movingAverage(SAMPLE_SIZE_TAPS);
  @Log private boolean stalling = false;

  @Log private double offset = ZERO_OFFSET;

  private final ElevatorSim sim =
      new ElevatorSim(
          DCMotor.getNEO(3),
          GEARING,
          Dimensions.CARRIAGE_MASS + Dimensions.FOREARM_MASS + Dimensions.CLAW_MASS,
          RELATIVE_CONVERSION.gearing(),
          MIN_HEIGHT,
          MAX_HEIGHT,
          true);

  @Log private boolean encoderUnplugged;
  private boolean stopped;

  private final Visualizer positionVisualizer;
  private final Visualizer setpointVisualizer;

  public Elevator(Visualizer positionVisualizer, Visualizer setpointVisualizer) {

    left.follow(lead);
    right.follow(lead);
    encoder.setDistancePerPulse(RELATIVE_CONVERSION.factor());
    // offsetEncoder.setPositionConversionFactor(ABSOLUTE_CONVERSION.factor());

    // for (int i = 0; i < 100; i++) System.out.println(offsetEncoder.getPosition());
    // if (Robot.isReal()) {
    //   offset = offsetEncoder.getPosition() + ZERO_OFFSET;
    // }

    // SmartDashboard.putNumber("start", offsetEncoder.getPosition());
    // STARTING POSITION ****MUST**** BE ABOVE THE ZERO LOCATION
    // VALUES NEAR THE RED TAPE MARKING ARE GOOD

    lead.burnFlash();
    left.burnFlash();
    right.burnFlash();

    this.positionVisualizer = positionVisualizer;
    this.setpointVisualizer = setpointVisualizer;

    setpoint = new State(getPosition(), 0, 0);
  }

  /** Returns the height of the elevator, in meters */
  @Log(name = "position")
  public double getPosition() {
    return Robot.isReal() ? encoder.getDistance() + offset : simEncoder.getDistance();
  }

  /** Sets the elevator's setpoint height */
  public Command setSetpoint(double height) {
    return runOnce(() -> setpoint = new State(height, 0, 0));
  }

  /** Returns the elevator setpoint as a {@link State} */
  public State getSetpoint() {
    return setpoint;
  }

  /** Follows a {@link TrapezoidProfile} to the desired height */
  public Command followProfile(double height) {
    var accel = new Derivative();

    return runOnce(accel::reset)
        .andThen(
            new DeferredCommand(
                () ->
                    new TrapezoidProfileCommand(
                        new TrapezoidProfile(
                            CONSTRAINTS,
                            new TrapezoidProfile.State(height, 0),
                            setpoint.trapezoidState()),
                        state ->
                            setpoint =
                                new State(
                                    state.position,
                                    state.velocity,
                                    accel.calculate(state.velocity)))))
        .withName("following profile");
  }

  /** Follows a {@link Trajectory} to the desired height */
  public Command followTrajectory(Trajectory trajectory) {
    return trajectory.follow(state -> setpoint = state, this).withName("following trajectory");
  }

  /**
   * If elevator is far enough from setpoint (we always use trapezoid profiling or trajectories), it
   * is dangerous. This method returns a debounced trigger for when the elbow encoder is likely
   * failing/not plugged in.
   */
  public Trigger onFailing() {
    return new Trigger(() -> encoderUnplugged).debounce(FAILING_DEBOUNCE_TIME);
  }

  public Command setStopped(boolean stopped) {
    return runOnce(() -> this.stopped = stopped);
  }

  public boolean stopped() {
    return stopped;
  }

  @Override
  public void periodic() {

    // if (bottomSwitch.get()) {
    //   offset = -encoder.getPosition();
    // }
    encoderUnplugged =
        encoder.getDistance() == 0
            && encoder.getRate() == 0
            && setpoint.position() != offset
            && lead.getAppliedOutput() != 0;

    setpoint =
        new State(
            MathUtil.clamp(setpoint.position(), MIN_HEIGHT, MAX_HEIGHT),
            setpoint.velocity(),
            setpoint.acceleration());

    double fbOutput = pid.calculate(getPosition(), setpoint.position());
    double ffOutput = ff.calculate(setpoint.velocity(), setpoint.acceleration());

    stalling = current.calculate(lead.getOutputCurrent()) >= CURRENT_THRESHOLD;

    lead.setVoltage(stopped ? 0 : ffOutput + fbOutput);

    positionVisualizer.setElevatorHeight(getPosition());
    setpointVisualizer.setElevatorHeight(setpoint.position());
  }

  @Override
  public void simulationPeriodic() {
    sim.setInputVoltage(lead.getAppliedOutput());
    sim.update(Constants.RATE);
    simEncoder.setDistance(sim.getPositionMeters());
  }

  @Override
  public void close() {
    lead.close();
    left.close();
    right.close();
    bottomSwitch.close();
    encoder.close();
  }
}
