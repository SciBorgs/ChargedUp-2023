package org.sciborgs1155.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import org.sciborgs1155.lib.Visualizer;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Constants.Dimensions;
import org.sciborgs1155.robot.Constants.Motors;
import org.sciborgs1155.robot.Ports.ElevatorPorts;

public class Elevator extends SubsystemBase implements Loggable {

  // Reference to a Mechanism2d for displaying the elevator's movement
  private final Visualizer visualizer;

  // this is all good
  private final CANSparkMax motor;
  private final RelativeEncoder encoder;

  private final ElevatorFeedforward ff;
  @Log private final ProfiledPIDController pid;

  // digital input
  @Log private final DigitalInput beambreak;
  @Log private final DigitalInput beambreakTwo;

  // goal height
  @Log private double height = 0;
  private double lastSpeed = 0;
  private double lastTime = Timer.getFPGATimestamp();

  // simulation
  private final ElevatorSim sim;

  public Elevator(Visualizer visualizer) {

    this.visualizer = visualizer;

    motor =
        Motors.ELEVATOR.buildCanSparkMaxGearbox(MotorType.kBrushless, ElevatorPorts.ELEVATOR_PORTS);
    encoder = motor.getEncoder();

    ff =
        new ElevatorFeedforward(
            Constants.Elevator.kS,
            Constants.Elevator.kG,
            Constants.Elevator.kV,
            Constants.Elevator.kA);
    pid =
        new ProfiledPIDController(
            Constants.Elevator.P,
            Constants.Elevator.I,
            Constants.Elevator.D,
            new Constraints(Constants.Elevator.maxVelocity, Constants.Elevator.maxAcceleration));

    beambreak = new DigitalInput(ElevatorPorts.BEAM_BREAK_PORTS[0]);
    beambreakTwo = new DigitalInput(ElevatorPorts.BEAM_BREAK_PORTS[1]);

    sim = new ElevatorSim(DCMotor.getNEO(3), 1, 10, 0.2, 0, Dimensions.ELEVATOR_HEIGHT, true);
  }

  @Config
  public void setTargetHeight(double newHeight) {
    height = newHeight;
  }

  @Override
  public void periodic() {
    if (!beambreak.get() || !beambreakTwo.get()) {
      double acceleration =
          (pid.getSetpoint().velocity - lastSpeed) / (Timer.getFPGATimestamp() - lastTime);
      double pidOutput = pid.calculate(encoder.getPosition(), height);
      double ffOutput = ff.calculate(pid.getSetpoint().velocity, acceleration);
      motor.setVoltage(pidOutput + ffOutput);

      lastSpeed = pid.getSetpoint().velocity;
    } else {
      motor.stopMotor();
    }
    lastTime = Timer.getFPGATimestamp();
  }

  @Override
  public void simulationPeriodic() {
    sim.setInputVoltage(motor.getAppliedOutput());
    sim.update(Constants.RATE);
    visualizer.setElevatorHeight(sim.getPositionMeters());
  }
}
