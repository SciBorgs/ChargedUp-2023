package org.sciborgs1155.robot.subsystems;

import static org.sciborgs1155.robot.Constants.Elevator.*;
import static org.sciborgs1155.robot.Ports.Elevator.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import org.sciborgs1155.lib.Derivative;
import org.sciborgs1155.lib.Trajectory;
import org.sciborgs1155.lib.Trajectory.State;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Constants.Dimensions;
import org.sciborgs1155.robot.util.Visualizer;

public class Elevator extends SubsystemBase implements Loggable, AutoCloseable {

  @Log(name = "applied output", methodName = "getAppliedOutput")
  private final CANSparkMax lead = MOTOR.build(MotorType.kBrushless, MIDDLE_MOTOR);

  private final CANSparkMax left = MOTOR.build(MotorType.kBrushless, LEFT_MOTOR);
  private final CANSparkMax right = MOTOR.build(MotorType.kBrushless, RIGHT_MOTOR);

  // @Log private final Encoder encoder = new Encoder(ENCODER[0], ENCODER[1]);
  // private final EncoderSim simEncoder = new EncoderSim(encoder);
  private final RelativeEncoder encoder = lead.getAlternateEncoder(Constants.THROUGHBORE_CPR);

  // private final AbsoluteEncoder offsetEncoder = right.getAbsoluteEncoder(Type.kDutyCycle);

  private final ElevatorFeedforward ff = new ElevatorFeedforward(FF.s(), FF.g(), FF.v(), FF.a());

  @Log
  @Log(name = "at setpoint", methodName = "atSetpoint")
  private final PIDController pid = new PIDController(PID.p(), PID.i(), PID.d());

  @Log(name = "position setpoint", methodName = "position")
  @Log(name = "velocity setpoint", methodName = "velocity")
  @Log(name = "acceleration setpoint", methodName = "acceleration")
  private State setpoint;

  private final LinearFilter filter = LinearFilter.movingAverage(SAMPLE_SIZE_TAPS);

  @Log private boolean hasSpiked = false;

  @Log private final double offset = 0.61842;

  private final ElevatorSim sim =
      new ElevatorSim(
          DCMotor.getNEO(3),
          GEARING,
          Dimensions.CARRIAGE_MASS + Dimensions.FOREARM_MASS + Dimensions.CLAW_MASS,
          RELATIVE_CONVERSION.gearing(),
          MIN_HEIGHT,
          MAX_HEIGHT,
          true);

  private final Visualizer positionVisualizer;
  private final Visualizer setpointVisualizer;

  public Elevator(Visualizer positionVisualizer, Visualizer setpointVisualizer) {
    left.follow(lead);
    right.follow(lead);

    encoder.setPositionConversionFactor(RELATIVE_CONVERSION.factor());
    encoder.setVelocityConversionFactor(RELATIVE_CONVERSION.factor() / 60.0);
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

    encoder.setPosition(offset);

    setpoint = new State(getPosition(), 0, 0);
  }

  /** Returns the height of the elevator, in meters */
  @Log(name = "position")
  public double getPosition() {
    return encoder.getPosition();
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
    var goal = new TrapezoidProfile.State(height, 0);
    var accel = new Derivative();

    return runOnce(accel::reset)
        .andThen(
            run(
                () -> {
                  var profile = new TrapezoidProfile(CONSTRAINTS, goal, setpoint.trapezoidState());
                  var target = profile.calculate(pid.getPeriod());
                  setpoint =
                      new State(target.position, target.velocity, accel.calculate(target.velocity));
                }))
        .until(() -> pid.atSetpoint() && goal.equals(setpoint.trapezoidState()));
  }

  public Command followTrajectory(Trajectory trajectory) {
    Timer timer = new Timer();
    return runOnce(timer::start)
        .andThen(run(() -> setpoint = trajectory.sample(timer.get())))
        .until(() -> setpoint.position() == trajectory.getLast());
  }

  @Override
  public void periodic() {
    double position = MathUtil.clamp(setpoint.position(), MIN_HEIGHT, MAX_HEIGHT);

    double fbOutput = pid.calculate(getPosition(), position);
    double ffOutput = ff.calculate(setpoint.velocity(), setpoint.acceleration());

    lead.setVoltage(fbOutput + ffOutput);

    hasSpiked = filter.calculate(lead.getOutputCurrent()) >= CURRENT_SPIKE_THRESHOLD;

    positionVisualizer.setElevatorHeight(getPosition());
    setpointVisualizer.setElevatorHeight(setpoint.position());
  }

  @Override
  public void simulationPeriodic() {
    sim.setInputVoltage(lead.getAppliedOutput());
    sim.update(Constants.RATE);
    encoder.setPosition(sim.getPositionMeters());
  }

  @Override
  public void close() {
    lead.close();
    left.close();
    right.close();
  }
}
