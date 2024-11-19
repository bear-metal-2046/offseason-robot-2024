package org.tahomarobotics.robot.elevator;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;


public class ElevatorConstants {

    public static final double GEAR_REDUCTION = 12d / 72d * 30d / 60d;
    private static final double MAIN_PULLEY_CIRCUMFERENCE = 0.22742; // Meters
    public static final double SENSOR_COEFFICIENT = GEAR_REDUCTION * MAIN_PULLEY_CIRCUMFERENCE;
    public static final double ELEVATOR_PHYSICAL_BOTTOM = Units.inchesToMeters(5.681); // Meters
    public static final double ELEVATOR_MAX_POSE = Units.inchesToMeters(52) - ELEVATOR_PHYSICAL_BOTTOM; // Meters
    public static final double ELEVATOR_MIN_POSE = 0.01; // Meters
    public static final double ELEVATOR_HIGH_POSE = Math.min(ELEVATOR_MAX_POSE, Units.inchesToMeters(60.5)) - ELEVATOR_PHYSICAL_BOTTOM; // Meters
    public static final double ELEVATOR_MID_POSE = Units.inchesToMeters(40.5) - ELEVATOR_PHYSICAL_BOTTOM; // Meters
    public static final double ELEVATOR_LOW_POSE = Units.inchesToMeters(14) - ELEVATOR_PHYSICAL_BOTTOM; // Meters
    public static final double POSITION_TOLERENCE = 0.005; //Meters

    public static final double ELEVATOR_MAX_VELOCITY = 2; // Meters / sec
    public static final double ELEVATOR_MAX_ACCELERATION = ELEVATOR_MAX_VELOCITY * 4.0; // Meters / sec^2


    static final TalonFXConfiguration elevatorConfig = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                    .withGravityType(GravityTypeValue.Elevator_Static)
                    .withKP(20.0)
                    .withKI(0.0)
                    .withKD(0.0)
                    .withKS(0.0)
                    .withKV(0.0)
                    .withKA(0.0)
                    .withKG(0.0)
            ).withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Coast)
                    .withInverted(InvertedValue.CounterClockwise_Positive))
            .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(ELEVATOR_MAX_VELOCITY)
                    .withMotionMagicAcceleration(ELEVATOR_MAX_ACCELERATION)
                    .withMotionMagicJerk(0.0))
            .withClosedLoopGeneral(new ClosedLoopGeneralConfigs(){{ContinuousWrap = false;}})
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1 / SENSOR_COEFFICIENT))
            .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true));
}
