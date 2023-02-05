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
import org.sciborgs1155.robot.Constants.PlacementConstants;
import org.sciborgs1155.robot.Ports.ElevatorPorts;

public class Elevator extends SubsystemBase implements Loggable {

  // Reference to a Mechanism2d for displaying the elevator's movement
  private final Visualizer visualizer;

  private final CANSparkMax lead, left, right;
  private final RelativeEncoder encoder;

  private final ElevatorFeedforward ff;
  @Log private final ProfiledPIDController pid;

  // digital input
  @Log private final DigitalInput beambreak;
  @Log private final DigitalInput beambreakTwo;

  @Log private final DigitalInput limitSwitchOne;
  @Log private final DigitalInput limitSwitchTwo;

  // goal height
  @Log private double height = 0;
  private double lastSpeed = 0;
  private double lastTime = Timer.getFPGATimestamp();

  // simulation
  private final ElevatorSim sim;

  public Elevator(Visualizer visualizer) {

    this.visualizer = visualizer;

    lead = Motors.ELEVATOR.build(MotorType.kBrushless, ElevatorPorts.MIDDLE_MOTOR);
    left = Motors.ELEVATOR.build(MotorType.kBrushless, ElevatorPorts.LEFT_MOTOR);
    right = Motors.ELEVATOR.build(MotorType.kBrushless, ElevatorPorts.RIGHT_MOTOR);
    left.follow(lead);
    right.follow(lead);
    encoder = lead.getEncoder();

    ff =
        new ElevatorFeedforward(
            PlacementConstants.Elevator.kS,
            PlacementConstants.Elevator.kG,
            PlacementConstants.Elevator.kV,
            PlacementConstants.Elevator.kA);
    pid =
        new ProfiledPIDController(
            PlacementConstants.Elevator.P,
            PlacementConstants.Elevator.I,
            PlacementConstants.Elevator.D,
            new Constraints(
                PlacementConstants.Elevator.maxVelocity,
                PlacementConstants.Elevator.maxAcceleration));

    beambreak = new DigitalInput(ElevatorPorts.BEAM_BREAK_PORTS[0]);
    beambreakTwo = new DigitalInput(ElevatorPorts.BEAM_BREAK_PORTS[1]);

    limitSwitchOne = new DigitalInput(ElevatorPorts.LIMIT_SWITCH_PORTS[0]);
    limitSwitchTwo = new DigitalInput(ElevatorPorts.LIMIT_SWITCH_PORTS[1]);

    sim = new ElevatorSim(DCMotor.getNEO(3), 1, 10, 0.2, 0, Dimensions.ELEVATOR_HEIGHT, true);
  }

  @Config
  public void setTargetHeight(double newHeight) {
    height = newHeight;
  }

  @Override
  public void periodic() {
    if (!beambreak.get() || !beambreakTwo.get() || !limitSwitchOne.get() || !limitSwitchOne.get()) {
      double acceleration =
          (pid.getSetpoint().velocity - lastSpeed) / (Timer.getFPGATimestamp() - lastTime);
      double pidOutput = pid.calculate(encoder.getPosition(), height);
      double ffOutput = ff.calculate(pid.getSetpoint().velocity, acceleration);
      lead.setVoltage(pidOutput + ffOutput);

      lastSpeed = pid.getSetpoint().velocity;
    } else {
      lead.stopMotor();
    }
    lastTime = Timer.getFPGATimestamp();

    visualizer.setElevatorHeight(encoder.getPosition());
  }

  @Override
  public void simulationPeriodic() {
    sim.setInputVoltage(lead.getAppliedOutput());
    sim.update(Constants.RATE);
    encoder.setPosition(sim.getPositionMeters());
  }
}
