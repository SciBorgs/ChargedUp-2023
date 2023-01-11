// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.sciborgs1155.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import org.sciborgs1155.lib.MotorConfig;
import org.sciborgs1155.robot.Constants.ModuleConstants;
import org.sciborgs1155.robot.Robot;

public class SwerveModule {
  private final CANSparkMax driveMotor; // Regular Neo
  private final CANSparkMax turnMotor; // Neo 550

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turningEncoder;

  private final PIDController m_drivePIDController =
      new PIDController(ModuleConstants.kPModuleDriveController, 0, 0);

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          ModuleConstants.kPModuleTurningController,
          0,
          0,
          new TrapezoidProfile.Constraints(
              ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
              ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

  /**
   * Constructs a SwerveModule.
   *
   * @param drivePort drive motor port
   * @param turnPort turning motor port
   * @param driveConfig MotorConfig for drive motor
   * @param turnConfig MotorConfig for turning motor
   */
  public SwerveModule(
      int drivePort, int turnPort, MotorConfig driveConfig, MotorConfig turnConfig) {
    driveMotor = driveConfig.buildCanSparkMax(MotorType.kBrushless, drivePort);
    turnMotor = turnConfig.buildCanSparkMax(MotorType.kBrushless, turnPort);

    driveEncoder = driveMotor.getEncoder();

    turningEncoder = turnMotor.getEncoder();

    // TODO proper conversion

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    // driveEncoder.setDistancePerPulse(ModuleConstants.kDriveEncoderDistancePerPulse);

    // Set the distance (in this case, angle) per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.
    // turningEncoder.setDistancePerPulse(ModuleConstants.kTurningEncoderDistancePerPulse);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    // add motors to rev physics sim
    if (Robot.isSimulation()) {
      REVPhysicsSim.getInstance().addSparkMax(driveMotor, DCMotor.getNEO(1));
      REVPhysicsSim.getInstance().addSparkMax(turnMotor, DCMotor.getNeo550(1));
    }
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // won't work until units are correct
    return new SwerveModuleState(
        driveEncoder.getVelocity(), new Rotation2d(turningEncoder.getPosition()));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        driveEncoder.getPosition(), new Rotation2d(turningEncoder.getPosition()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(turningEncoder.getPosition()));

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(driveEncoder.getVelocity(), state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turningPIDController.calculate(turningEncoder.getPosition(), state.angle.getRadians());

    // Calculate the turning motor output from the turning PID controller.
    driveMotor.set(driveOutput);
    turnMotor.set(turnOutput);
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    driveEncoder.setPosition(0);
    turningEncoder.setPosition(0);
  }
}
