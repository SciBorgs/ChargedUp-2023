package org.sciborgs1155.robot.subsystems;

import static org.sciborgs1155.robot.Constants.Elevator.*;
import static org.sciborgs1155.robot.Ports.Elevator.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import org.sciborgs1155.lib.Derivative;
import org.sciborgs1155.lib.Trajectory;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Constants.Dimensions;
import org.sciborgs1155.robot.util.Visualizer;

public class Elevator extends SubsystemBase implements Loggable, AutoCloseable {

  @Log(name = "applied output", methodName = "getAppliedOutput")
  private final CANSparkMax lead = MOTOR.build(MotorType.kBrushless, MIDDLE_MOTOR);

  private final CANSparkMax left = MOTOR.build(MotorType.kBrushless, LEFT_MOTOR);
  private final CANSparkMax right = MOTOR.build(MotorType.kBrushless, RIGHT_MOTOR);

  @Log private final Encoder encoder = new Encoder(ENCODER[0], ENCODER[1]);
  private final EncoderSim simEncoder = new EncoderSim(encoder);

  private final AbsoluteEncoder offsetEncoder = lead.getAbsoluteEncoder(Type.kDutyCycle);

  private final ElevatorFeedforward ff = new ElevatorFeedforward(FF.s(), FF.g(), FF.v(), FF.a());

  @Log
  @Log(name = "at setpoint", methodName = "atSetpoint")
  private final PIDController pid = new PIDController(PID.p(), PID.i(), PID.d());

  private Vector<N3> setpoint;

  private final LinearFilter filter = LinearFilter.movingAverage(SAMPLE_SIZE_TAPS);

  @Log private boolean hasSpiked = false;

  @Log private double offset = 0.61842;

  private final ElevatorSim sim =
      new ElevatorSim(
          DCMotor.getNEO(3),
          RELATIVE_CONVERSION.gearing(),
          Dimensions.ELEVATOR_MASS + Dimensions.FOREARM_MASS + Dimensions.CLAW_MASS,
          RELATIVE_CONVERSION.units(),
          Dimensions.ELEVATOR_MIN_HEIGHT,
          Dimensions.ELEVATOR_MAX_HEIGHT,
          true);

  private final Visualizer positionVisualizer;
  private final Visualizer setpointVisualizer;

  public Elevator(Visualizer positionVisualizer, Visualizer setpointVisualizer) {
    left.follow(lead);
    right.follow(lead);

    encoder.setDistancePerPulse(RELATIVE_CONVERSION.factor());
    offsetEncoder.setPositionConversionFactor(ABSOLUTE_CONVERSION.factor());

    // for (int i = 0; i < 100; i++) System.out.println(offsetEncoder.getPosition());
    // if (Robot.isReal()) {
    //   offset = offsetEncoder.getPosition() + ZERO_OFFSET;
    // }

    SmartDashboard.putNumber("start", offsetEncoder.getPosition());
    // STARTING POSITION ****MUST**** BE ABOVE THE ZERO LOCATION
    // VALUES NEAR THE RED TAPE MARKING ARE GOOD

    lead.burnFlash();
    left.burnFlash();
    right.burnFlash();

    this.positionVisualizer = positionVisualizer;
    this.setpointVisualizer = setpointVisualizer;

    pid.setSetpoint(getPosition());
  }

  /** Returns the height of the elevator, in meters */
  @Log(name = "position")
  public double getPosition() {
    return encoder.getDistance() + offset;
  }

  /** Returns the goal of the elevator, in meters */
  public boolean atSetpoint() {
    return pid.atSetpoint();
  }

  /** Sets the elevator's setpoint height */
  public void setSetpoint(Vector<N3> state) {
    prevSetpoint = setpoint;
    setpoint = state;
  }

  /** Returns a trapezoid profile state from our current setpoint */
  public TrapezoidProfile.State getTrapezoidSetpoint() {
    return new TrapezoidProfile.State(setpoint.get(0, 0), setpoint.get(1, 0));
  }

  /** Follows a {@link TrapezoidProfile} to the desired height */
  public Command followProfile(double height) {
    var goal = new TrapezoidProfile.State(height, 0);
    var accel = new Derivative();
    return run(() -> {
          var profile = new TrapezoidProfile(CONSTRAINTS, goal, getTrapezoidSetpoint());
          var setpoint = profile.calculate(pid.getPeriod());
          this.setpoint =
              VecBuilder.fill(
                  setpoint.position, setpoint.velocity, accel.calculate(setpoint.velocity));
        })
        .until(() -> atSetpoint() && goal.equals(getTrapezoidSetpoint()));
  }

  public Command followTrajectory(Trajectory trajectory) {
    Timer timer = new Timer();
    return runOnce(timer::start)
        .andThen(
            run(() -> setpoint = trajectory.sample(timer.get()))
                .until(() -> timer.hasElapsed(trajectory.getTotalTime())));
  }

  @Override
  public void periodic() {
    double fbOutput = pid.calculate(getPosition(), setpoint.get(0, 0));
    double ffOutput = ff.calculate(setpoint.get(1, 0), setpoint.get(2, 0));

    lead.setVoltage(fbOutput + ffOutput);

    hasSpiked = filter.calculate(lead.getOutputCurrent()) >= CURRENT_SPIKE_THRESHOLD;

    positionVisualizer.setElevatorHeight(getPosition());
    setpointVisualizer.setElevatorHeight(setpoint.get(0, 0));
  }

  @Override
  public void simulationPeriodic() {
    sim.setInputVoltage(lead.getAppliedOutput());
    sim.update(Constants.RATE);
    simEncoder.setDistance(sim.getPositionMeters());
    simEncoder.setRate(sim.getVelocityMetersPerSecond());
  }

  @Override
  public void close() {
    lead.close();
    left.close();
    right.close();
  }
}
