package org.tahomarobotics.robot.chassis;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;

public class ChassisConstants {

    public static final double TRACK_WIDTH = 0.5816;
    public static final double WHEELBASE = 0.8194;
    public static final double HALF_TRACK_WIDTH = TRACK_WIDTH / 2;
    public static final double HALF_WHEELBASE = WHEELBASE / 2;

    public static final double WHEEL_RADIUS = 0.0508;
    public static final double WHEEL_CIRCUMFERENCE = 2 * Math.PI * WHEEL_RADIUS;
    public static final double DRIVE_REDUCTION = 1;
    public static final double DRIVE_POSITION_COEFFICIENT = WHEEL_CIRCUMFERENCE * DRIVE_REDUCTION;

    public static final DCMotor SWERVE_MOTOR = DCMotor.getKrakenX60(1);
    public static final double MAX_VELOCITY = SWERVE_MOTOR.freeSpeedRadPerSec * DRIVE_REDUCTION * WHEEL_RADIUS * 1.1;
    public static final double ACCELERATION_LIMIT = 6.0;

    public static final Translation2d FRONT_LEFT_OFFSET = new Translation2d(HALF_WHEELBASE, HALF_TRACK_WIDTH);
    public static final Translation2d FRONT_RIGHT_OFFSET = new Translation2d(HALF_WHEELBASE, -HALF_TRACK_WIDTH);
    public static final Translation2d BACK_LEFT_OFFSET = new Translation2d(-HALF_WHEELBASE, HALF_TRACK_WIDTH);
    public static final Translation2d BACK_RIGHT_OFFSET = new Translation2d(-HALF_WHEELBASE, -HALF_TRACK_WIDTH);

    public static final TalonFXConfiguration driveMotorConfiguration = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.Clockwise_Positive))
            .withAudio(new AudioConfigs()
                    .withBeepOnBoot(true)
                    .withBeepOnConfig(true));

    public static final TalonFXConfiguration steerMotorConfiguration = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.Clockwise_Positive))
            .withAudio(new AudioConfigs()
                    .withBeepOnBoot(true)
                    .withBeepOnConfig(true));

    public static final MagnetSensorConfigs encoderConfiguration = new MagnetSensorConfigs()
            .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
            .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);

    public static double clampAccel(double value) {
        return MathUtil.clamp(value, -ACCELERATION_LIMIT, ACCELERATION_LIMIT);
    }
}
