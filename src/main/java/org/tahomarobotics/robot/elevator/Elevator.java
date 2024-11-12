package org.tahomarobotics.robot.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Commands;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.Robot;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.tahomarobotics.robot.util.SubsystemIF;

import static org.tahomarobotics.robot.elevator.ElevatorConstants.*;

public class Elevator extends SubsystemIF {

    public static final Logger logger = LoggerFactory.getLogger(Elevator.class);
    private static final Elevator INSTANCE = new Elevator();
    private double targetHeight;
    private final MotionMagicVoltage positionControl = new MotionMagicVoltage(0.0).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);
    TalonFX elevatorRight;
    TalonFX elevatorLeft;

    //Rotations
    private final StatusSignal<Angle> motorPosition;
    private final StatusSignal<AngularVelocity> elevatorVelocity;
    private final StatusSignal<Current> elevatorCurrent;

    public static Elevator getInstance() {
        return INSTANCE;
    }

    private Elevator() {
        RobustConfigurator configurator = new RobustConfigurator(logger);

        elevatorRight = new TalonFX(RobotMap.ELEVATOR_RIGHT_MOTOR);
        elevatorLeft = new TalonFX(RobotMap.ELEVATOR_LEFT_MOTOR);

        configurator.configureTalonFX(elevatorRight, elevatorConfig, elevatorLeft, false);

        motorPosition = elevatorRight.getPosition();
        elevatorVelocity = elevatorRight.getVelocity();
        elevatorCurrent = elevatorRight.getStatorCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(RobotConfiguration.MECHANISM_UPDATE_FREQUENCY, elevatorCurrent, motorPosition, elevatorVelocity);

        ParentDevice.optimizeBusUtilizationForAll(elevatorRight, elevatorLeft);

    }

    public enum ElevatorStates {
        HIGH,
        MID,
        LOW
    }

    //Degrees
    public double getElevatorMotorRotation() {
        return motorPosition.getValueAsDouble() * ROTATIONS_TO_DEGREES;
    }

    public double getElevatorHeight() {
        return getElevatorMotorRotation() * DEGREES_TO_METERS;
    }

    public double getElevatorPos(ElevatorStates elevatorState) {
        return switch (elevatorState) {
            case MID -> ELEVATOR_MID_POSE;
            case HIGH -> ELEVATOR_HIGH_POSE;
            default -> ELEVATOR_LOW_POSE;
        };
    }

    public void setElevatorHeight(double height) {
        targetHeight = MathUtil.clamp(height, ELEVATOR_MIN_POSE, ELEVATOR_MAX_POSE);
        elevatorRight.setControl(positionControl.withPosition(targetHeight * METERS_TO_ROTATIONS));
    }

    public boolean isAtPosition() {
        return Math.abs(targetHeight - getElevatorHeight()) <= ElevatorConstants.POSITION_TOLERENCE;
    }

    public void setVelocity(double speed) {
        setElevatorHeight(getElevatorHeight() + speed * Robot.kDefaultPeriod);
    }

    public void stop() {
        elevatorRight.stopMotor();
    }

    @Override
    public SubsystemIF initialize() {
        return this;
    }

    @Override
    public double getEnergyUsed() {
        return 0;
    }

    @Override
    public double getTotalCurrent() {
        return 0;
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(motorPosition, elevatorVelocity, elevatorCurrent);
    }

    @Override
    public void onTeleopInit() {
        Commands.waitUntil(RobotState::isEnabled)
                .andThen(Commands.runOnce(() -> {
                    setElevatorHeight(ELEVATOR_LOW_POSE);
                }))
                .ignoringDisable(true).schedule();
    }
}