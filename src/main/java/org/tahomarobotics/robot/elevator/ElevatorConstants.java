package org.tahomarobotics.robot.elevator;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


public class ElevatorConstants {

    public static final double GEAR_REDUCTION = 12d / 72d * 30d / 60d;
    private static final double MAIN_PULLEY_CIRCUMFERENCE = 0.22742; // Meters
    public static final double SENSOR_COEFFICIENT = GEAR_REDUCTION / 2048 * MAIN_PULLEY_CIRCUMFERENCE;
    public static final double ELEVATOR_HIGH_POSE = 1.4; // Meters()
    public static final double ELEVATOR_MID_POSE = 1.4/2; // Meters()
    public static final double ELEVATOR_LOW_POSE = 0.2; // Meters()
    public static final double ELEVATOR_MAX_POSE = 1.4; // Meters()
    public static final double ELEVATOR_MIN_POSE = 0.0; // Meters()
    public static final double VELOCITY_EPSILON = 0.005;
    public static final double POSITION_EPSILON = 0.025;

    //rotations = meters/(circumference * GR)
    public static final double METERS_TO_ROTATIONS = 1.0 / (MAIN_PULLEY_CIRCUMFERENCE * GEAR_REDUCTION);
    //technically not necessary but better to read
    public static final double DEGREES_TO_METERS = (MAIN_PULLEY_CIRCUMFERENCE * GEAR_REDUCTION) / 360.0;
    public static final double ROTATIONS_TO_DEGREES = 1.0 / 360.0;

    public static final double ELEVATOR_MAX_VELOCITY = 0.1; // Meters / sec()
    public static final double ELEVATOR_MAX_ACCELERATION = 0.05; // Meters / sec^2 ()


    static final TalonFXConfiguration elevatorConfig = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                    .withGravityType(GravityTypeValue.Elevator_Static)
                    .withKP(1.0)
                    .withKD(0.0)
                    .withKS(0.0)
                    .withKV(0.0)
                    .withKA(0.0)
                    .withKG(0.0)
            ).withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.Clockwise_Positive))
            .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(ELEVATOR_MAX_VELOCITY)
                    .withMotionMagicAcceleration(ELEVATOR_MAX_ACCELERATION)
                    .withMotionMagicJerk(0.0))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(SENSOR_COEFFICIENT))
            .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true));
}
