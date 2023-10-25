package org.sciborgs1155.robot.arm;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import org.sciborgs1155.lib.constants.ArmFFConstants;
import org.sciborgs1155.lib.constants.ElevatorFFConstants;
import org.sciborgs1155.lib.constants.MotorConfig;
import org.sciborgs1155.lib.constants.MotorConfig.NeutralBehavior;
import org.sciborgs1155.lib.constants.PIDConstants;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Constants.Dimensions;
import org.sciborgs1155.robot.arm.ElevatorIO.ElevatorConfig;
import org.sciborgs1155.robot.arm.JointIO.JointConfig;

public class ArmConstants {

  public static final class Wrist {
    public static final MotorConfig MOTOR_CFG =
        MotorConfig.base().withNeutralBehavior(NeutralBehavior.BRAKE);

    public static final double MOTOR_GEARING = 53.125 / 1.0;
    // Gearing for motor : angle (radians)

    public static final double CONVERSION_RELATIVE = 2.0 * Math.PI / Constants.THROUGHBORE_PPR;
    public static final double CONVERSION_ABS = -2.0 * Math.PI;

    public static final PIDConstants PID_NEW = new PIDConstants(5.5, 0, 0.1);
    public static final PIDConstants PID_OLD = new PIDConstants(5.5, 0, 0.1);
    public static final ArmFFConstants FF_NEW =
        new ArmFFConstants(0.34613, 0.25692, 0.78381, 0.090836);
    public static final ArmFFConstants FF_OLD = new ArmFFConstants(0.1542, 0.6, 0.91, 0.038046);

    public static final Constraints CONSTRAINTS = new Constraints(2.45, 3.6);

    public static final double MAX_ANGLE = Math.PI;
    public static final double MIN_ANGLE = -Math.PI;

    public static final double ZERO_OFFSET = 0.885; // duty cycle

    public static final JointConfig CONFIG_OLD =
        new JointConfig(
            FF_OLD,
            PID_OLD,
            DCMotor.getNEO(1),
            MOTOR_GEARING,
            Dimensions.CLAW_LENGTH_OLD,
            Dimensions.CLAW_MASS_OLD,
            MIN_ANGLE,
            MAX_ANGLE);
    public static final JointConfig CONFIG_NEW =
        new JointConfig(
            FF_NEW,
            PID_NEW,
            DCMotor.getNEO(1),
            MOTOR_GEARING,
            Dimensions.CLAW_LENGTH,
            Dimensions.CLAW_MASS,
            MIN_ANGLE,
            MAX_ANGLE);
  }

  public static final class Elbow {
    public static final MotorConfig MOTOR_CFG =
        MotorConfig.base().withNeutralBehavior(NeutralBehavior.BRAKE).withCurrentLimit(50);

    public static final double MOTOR_GEARING = 63.75 / 1.0;

    public static final double ENCODER_RATIO = 12.0 / 72.0;
    public static final double CONVERSION = ENCODER_RATIO * 2 * Math.PI / Constants.THROUGHBORE_PPR;

    public static final PIDConstants PID_NEW = new PIDConstants(12, 0, 1.1); // d = 2.18954
    public static final PIDConstants PID_OLD = new PIDConstants(12, 0, 1.1); // d = 2.18954
    public static final ArmFFConstants FF_NEW =
        new ArmFFConstants(0.06403, 0.50715, 1.3482, 0.049377);
    public static final ArmFFConstants FF_OLD = new ArmFFConstants(0.020283, 0.71, 1.3174, 0.20891);

    public static final Constraints CONSTRAINTS = new Constraints(2.9, 3.4);

    public static final double MAX_ANGLE = 3.0 * Math.PI / 2.0;
    public static final double MIN_ANGLE = -Math.PI / 2.0;

    public static final JointConfig CONFIG_OLD =
        new JointConfig(
            FF_OLD,
            PID_OLD,
            DCMotor.getNEO(3),
            MOTOR_GEARING,
            Dimensions.FOREARM_LENGTH,
            Dimensions.FOREARM_MASS + Dimensions.CLAW_MASS_OLD,
            MIN_ANGLE,
            MAX_ANGLE);

    public static final JointConfig CONFIG_NEW =
        new JointConfig(
            FF_NEW,
            PID_NEW,
            DCMotor.getNEO(3),
            MOTOR_GEARING,
            Dimensions.FOREARM_LENGTH,
            Dimensions.FOREARM_MASS + Dimensions.CLAW_MASS,
            MIN_ANGLE,
            MAX_ANGLE);

    public static final double OFFSET = -1.248660;

    public static final double FAILING_DEBOUNCE_TIME = 0.2;
  }

  public static final class Elevator {
    public static final MotorConfig MOTOR_CFG =
        MotorConfig.base().withNeutralBehavior(NeutralBehavior.BRAKE).withCurrentLimit(35);

    public static final double MOTOR_GEARING = 30.0 / 1.0;
    // Gearing for motor : height (meters)

    public static final double SPROCKET_RADIUS = 0.0181864;
    // Radius of the sprocket in meters

    public static final double SPROCKET_CIRCUMFERENCE = 2.0 * Math.PI * SPROCKET_RADIUS;
    // Circumference of the sprocket in meters (2 * π * R)

    public static final double CONVERSION_RELATIVE =
        SPROCKET_CIRCUMFERENCE / Constants.THROUGHBORE_PPR;

    public static final PIDConstants PID = new PIDConstants(55, 0, 0.5);
    public static final ElevatorFFConstants FF =
        new ElevatorFFConstants(0.4, 0.069335, 33.25, 1.5514);

    public static final double MIN_HEIGHT = 0;
    public static final double MAX_HEIGHT = 0.65;

    public static final ElevatorConfig CONFIG =
        new ElevatorConfig(
            FF,
            PID,
            DCMotor.getNEO(3),
            MOTOR_GEARING,
            Dimensions.CARRIAGE_MASS + Dimensions.FOREARM_MASS + Dimensions.CLAW_MASS,
            SPROCKET_RADIUS,
            MIN_HEIGHT,
            MAX_HEIGHT);

    public static final int SAMPLE_SIZE_TAPS = 3;
    public static final int CURRENT_THRESHOLD = 35;

    public static final Constraints CONSTRAINTS = new Constraints(3, 3.35);

    public static final double ZERO_OFFSET = 0.61842;

    public static final double FAILING_DEBOUNCE_TIME = 0.2;
  }
}
