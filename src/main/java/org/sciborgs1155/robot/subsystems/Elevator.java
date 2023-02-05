package org.sciborgs1155.robot.subsystems;

import static org.sciborgs1155.robot.Constants.Elevator.*;
import static org.sciborgs1155.robot.Ports.Elevator.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import org.sciborgs1155.lib.Derivative;
import org.sciborgs1155.lib.Visualizer;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Constants.Dimensions;
import org.sciborgs1155.robot.Constants.Motors;

public class Elevator extends SubsystemBase implements Loggable {

  private final CANSparkMax lead, left, right;
  private final RelativeEncoder encoder;

  private final ElevatorFeedforward ff;
  @Log private final ProfiledPIDController pid;

  // digital input
  @Log private final DigitalInput beambreak;
  @Log private final DigitalInput beambreakTwo;

  @Log private final DigitalInput limitSwitchOne;
  @Log private final DigitalInput limitSwitchTwo;

  @Log private double targetHeight = 0;

  private final Derivative accel;

  // simulation
  private final ElevatorSim sim;

  public Elevator() {
    lead = Motors.ELEVATOR.build(MotorType.kBrushless, MIDDLE_MOTOR);
    left = Motors.ELEVATOR.build(MotorType.kBrushless, LEFT_MOTOR);
    right = Motors.ELEVATOR.build(MotorType.kBrushless, RIGHT_MOTOR);
    left.follow(lead);
    right.follow(lead);
    encoder = lead.getEncoder();

    ff = new ElevatorFeedforward(kS, kG, kV, kA);
    pid = new ProfiledPIDController(kP, kI, kD, CONSTRAINTS);

    beambreak = new DigitalInput(BEAM_BREAK_PORTS[0]);
    beambreakTwo = new DigitalInput(BEAM_BREAK_PORTS[1]);

    limitSwitchOne = new DigitalInput(LIMIT_SWITCH_PORTS[0]);
    limitSwitchTwo = new DigitalInput(LIMIT_SWITCH_PORTS[1]);

    accel = new Derivative();

    sim =
        new ElevatorSim(
            DCMotor.getNEO(3),
            10,
            4,
            Units.inchesToMeters(2),
            Dimensions.ELEVATOR_MIN_HEIGHT,
            Dimensions.ELEVATOR_MAX_HEIGHT,
            true);
  }

  public double getHeight() {
    return encoder.getPosition();
  }

  public double getTargetHeight() {
    return targetHeight;
  }

  public boolean isHitting() {
    // return beambreak.get() || beambreakTwo.get() || limitSwitchOne.get() || limitSwitchOne.get();
    return false;
  }

  public Command setTargetHeight(double targetHeight) {
    return runOnce(() -> this.targetHeight = targetHeight);
  }

  @Override
  public void periodic() {
    double pidOutput = pid.calculate(encoder.getPosition(), targetHeight);
    double ffOutput =
        ff.calculate(pid.getSetpoint().velocity, accel.calculate(pid.getSetpoint().velocity));

    lead.setVoltage(isHitting() ? 0 : pidOutput + ffOutput);

    Visualizer.getInstance().setElevatorHeight(encoder.getPosition());
  }

  @Override
  public void simulationPeriodic() {
    sim.setInputVoltage(lead.getAppliedOutput());
    sim.update(Constants.RATE);
    encoder.setPosition(sim.getPositionMeters());
  }
}
