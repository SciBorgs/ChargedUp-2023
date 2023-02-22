package org.sciborgs1155.robot.subsystems;

import static org.sciborgs1155.robot.Constants.Elevator.*;
import static org.sciborgs1155.robot.Ports.Elevator.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
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

public class Elevator extends SubsystemBase implements Loggable, AutoCloseable {

  @Log(name = "applied output", methodName = "getAppliedOutput")
  private final CANSparkMax lead = Motors.ELEVATOR.build(MotorType.kBrushless, MIDDLE_MOTOR);

  private final CANSparkMax left = Motors.ELEVATOR.build(MotorType.kBrushless, LEFT_MOTOR);
  private final CANSparkMax right = Motors.ELEVATOR.build(MotorType.kBrushless, RIGHT_MOTOR);

  @Log private final Encoder encoder = new Encoder(ENCODER[0], ENCODER[1]);
  private final EncoderSim simEncoder = new EncoderSim(encoder);

  private final ElevatorFeedforward ff = new ElevatorFeedforward(kS, kG, kV, kA);

  @Log private final ProfiledPIDController pid = new ProfiledPIDController(kP, kI, kD, CONSTRAINTS);

  @Log(name = "acceleration", methodName = "getLastOutput")
  private final Derivative accel = new Derivative();

  private final ElevatorSim sim =
      new ElevatorSim(
          DCMotor.getNEO(3),
          CONVERSION,
          Dimensions.ELEVATOR_MASS,
          SPROCKET_RADIUS,
          Dimensions.ELEVATOR_MIN_HEIGHT,
          Dimensions.ELEVATOR_MAX_HEIGHT,
          true);

  private final Visualizer visualizer;

  public Elevator(Visualizer visualizer) {
    left.follow(lead);
    right.follow(lead);

    encoder.setDistancePerPulse(ENCODER_FACTOR);

    lead.burnFlash();
    left.burnFlash();
    right.burnFlash();

    this.visualizer = visualizer;
  }

  /** Returns the height of the elevator, in meters */
  @Log(name = "position")
  public double getPosition() {
    return encoder.getDistance();
  }

  /** Returns the goal of the elevator, in meters */
  @Log(name = "at goal")
  public boolean atGoal() {
    return pid.atGoal();
  }

  /** Sets the elevator's goal to a {@link TrapezoidProfile.State} */
  public Command setGoal(TrapezoidProfile.State goal) {
    return runOnce(
        () ->
            pid.setGoal(
                new TrapezoidProfile.State(
                    MathUtil.clamp(
                        goal.position,
                        Dimensions.ELEVATOR_MIN_HEIGHT,
                        Dimensions.ELEVATOR_MAX_HEIGHT),
                    goal.velocity)));
  }

  /** Runs the elevator to a goal {@link TrapezoidProfile.State} */
  public Command runToGoal(TrapezoidProfile.State goal) {
    return setGoal(goal).andThen(Commands.waitUntil(this::atGoal));
  }

  @Override
  public void periodic() {
    double fbOutput = pid.calculate(getPosition());
    double ffOutput =
        ff.calculate(pid.getSetpoint().velocity, accel.calculate(pid.getSetpoint().velocity));

    lead.setVoltage(fbOutput + ffOutput);

    visualizer.setElevator(getPosition(), pid.getGoal().position);
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
