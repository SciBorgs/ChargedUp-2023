package org.sciborgs1155.robot.subsystems;

import static org.sciborgs1155.robot.Constants.Elevator.*;
import static org.sciborgs1155.robot.Ports.Elevator.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.annotations.Log;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Constants.Dimensions;
import org.sciborgs1155.robot.Constants.Motors;

public class Elevator extends SubsystemBase {

  @Log(name = "applied output", methodName = "getAppliedOutput")
  private final CANSparkMax lead = Motors.ELEVATOR.build(MotorType.kBrushless, MIDDLE_MOTOR);

  private final CANSparkMax left = Motors.ELEVATOR.build(MotorType.kBrushless, LEFT_MOTOR);
  private final CANSparkMax right = Motors.ELEVATOR.build(MotorType.kBrushless, RIGHT_MOTOR);

  private final RelativeEncoder encoder = lead.getAlternateEncoder(Constants.THROUGH_BORE_CPR);

  private final ElevatorFeedforward feedforward = new ElevatorFeedforward(kS, kG, kV, kA);

  private final ProfiledPIDController feedback = new ProfiledPIDController(kP, kI, kD, CONSTRAINTS);

  private final ElevatorSim sim =
      new ElevatorSim(
          DCMotor.getNEO(3),
          1 / ENCODER_POSITION_FACTOR,
          Dimensions.ELEVATOR_MASS,
          SPROCKET_RADIUS,
          Dimensions.ELEVATOR_MIN_HEIGHT,
          Dimensions.ELEVATOR_MAX_HEIGHT,
          true);

  public Elevator() {
    left.follow(lead);
    right.follow(lead);

    encoder.setPositionConversionFactor(ENCODER_POSITION_FACTOR);
    encoder.setVelocityConversionFactor(ENCODER_VELOCITY_FACTOR);

    lead.burnFlash();
    left.burnFlash();
    right.burnFlash();
  }

  public double getPosition() {
    return encoder.getPosition();
  }

  public Command setGoal(TrapezoidProfile.State goal) {
    return runOnce(() -> ));
  }

  @Override
  public void periodic() {
    double fb = fb.
  }

  @Override
  public void simulationPeriodic() {
    // TODO Auto-generated method stub
    super.simulationPeriodic();
  }
}
