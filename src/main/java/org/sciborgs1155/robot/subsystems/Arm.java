package org.sciborgs1155.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import org.sciborgs1155.robot.Constants.Arm.Elbow;
import org.sciborgs1155.robot.Constants.Arm.Wrist;
import org.sciborgs1155.robot.Ports;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import org.sciborgs1155.lib.Derivative;

public class Arm extends SubsystemBase implements Loggable {

    private final ArmFeedforward wristFeedforward = new ArmFeedforward(Elbow.kS, Elbow.kG, Elbow.kV, Elbow.kA);

    private final ArmFeedforward elbowFeedForward = new ArmFeedforward(Elbow.kS, Elbow.kG, Elbow.kV, Elbow.kA);

    @Log(name = "Elbow Feedback")
    private final ProfiledPIDController elbowFeedback = new ProfiledPIDController(Elbow.kP, Elbow.kI, Elbow.kD, Elbow.CONSTRAINTS);

    @Log(name = "Wrist Feedback")
    private final ProfiledPIDController wristFeedback = new ProfiledPIDController(Wrist.kP, Wrist.kI, Wrist.kD, Wrist.CONSTRAINTS);

    @Log(name = "Wrist Acceleration", methodName = "getLastOutput")
    private final Derivative wristAccel = new Derivative();
  
    @Log(name = "Elbow Acceleration", methodName = "getLastOutput")
    private final Derivative elbowAccel = new Derivative();

    private final CANSparkMax elbowLead, elbowLeft, elbowRight, wrist;
    private final RelativeEncoder elbowEncoder;
    private final AbsoluteEncoder wristEncoder;

    public Arm() {
        this.elbowLead = new CANSparkMax(0, null);
    }

    public Rotation2d getElbowPosition() {
        return elbowEncoder.getPosition();
    }

    /** Wrist position relative to the forearm */
    public Rotation2d getRelativeWristPosition() {
        return wrist.getPosition();
    }

    /** Wrist position relative to chassis */
    @Log(name = "wrist absolute positon", methodName = "getDegrees")
    public Rotation2d getAbsoluteWristPosition() {
        return getRelativeWristPosition().plus(getElbowPosition());
    }

    /** Elbow goal relative to the chassis */
    public Rotation2d getElbowGoal() {
        return Rotation2d.fromRadians(elbowFeedback.getGoal().position);
    }

    /** Wrist goal relative to forearm */
    public Rotation2d getRelativeWristGoal() {
        return Rotation2d.fromRadians(wristFeedback.getGoal().position);
    }

    /** Wrist goal relative to the chassis */
    public Rotation2d getAbsoluteWristGoal() {
        return getRelativeWristGoal().plus(getElbowGoal());
    }

    @Override
    default void initSendable(SendableBuilder builder) {
      builder.addDoubleProperty("position", () -> getPosition().getDegrees(), null);
      builder.addDoubleProperty("velocity", this::getVelocity, null);
      builder.addDoubleProperty("current drawn", this::getCurrentDrawn, null);
    }
}
