package org.tahomarobotics.robot.elevator;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


public class ElevatorConstants {

    public static final double GEAR_REDUCTION = 12d/72d * 30d/60d;
    private static final double MAIN_PULLEY_CIRCUMFERENCE = 0.22742; // Meters
    public static final double SENSOR_COEFFICIENT = GEAR_REDUCTION / 2048 * MAIN_PULLEY_CIRCUMFERENCE;
    public static final double ELEVATOR_MAX = 1.4; // Meters()
    public static final double ELEVATOR_MAX_VELOCITY = 0.0; // Meters / sec()
    public static final double ELEVATOR_MAX_ACCELERATION = 1.0; // Meters / sec^2 ()
//    private static final double REFERENCE_VOLTAGE = 12;
//    private static final double VELOCITY_CURRENT_LIMIT = 80; // amps
//    private static final double ACCEL_CURRENT_LIMIT = 60; // amps
//
//
//    // Feed Forward
//    public static final double S_VOLTS = 0.0; // Static Gain
//    public static final double COS_VOLTS = 0.0; // Gravity Gain
//    public static final double V_VOLT_SECOND_PER_RAD = 0.0; // Velocity gain;
//    public static final double A_VOLT_SECOND_SQUARED_PER_RAD = 0.0; // Acceleration Gain
//
//    // Feed Back
//    public static final double kP = 1.0;
//    public static final double kI = 0.0;
//    public static final double kD = 0.0;

    static final TalonFXConfiguration elevatorConfig = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                    .withGravityType(GravityTypeValue.Elevator_Static)
                    .withKP(0.0)
                    .withKD(0.0)
                    .withKS(0.0)
                    .withKV(0.0)
                    .withKA(0.0)
                    .withKG(0.0)
            ).withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.Clockwise_Positive))
            .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(0.0)
                    .withMotionMagicAcceleration(0.0)
                    .withMotionMagicJerk(0.0))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(SENSOR_COEFFICIENT))
            .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true));

    static final TalonFXConfiguration followerConfig = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                    .withGravityType(GravityTypeValue.Elevator_Static)
                    .withKP(0.0)
                    .withKD(0.0)
                    .withKS(0.0)
                    .withKV(0.0)
                    .withKA(0.0)
                    .withKG(0.0)
            ).withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.CounterClockwise_Positive))
            .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(0.0)
                    .withMotionMagicAcceleration(0.0)
                    .withMotionMagicJerk(0.0))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(SENSOR_COEFFICIENT))
            .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true));
}
