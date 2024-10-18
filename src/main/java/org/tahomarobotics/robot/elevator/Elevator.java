package org.tahomarobotics.robot.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Commands;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.tahomarobotics.robot.util.SubsystemIF;

import static org.tahomarobotics.robot.elevator.ElevatorConstants.*;

public class Elevator extends SubsystemIF {

    public static final Logger logger = LoggerFactory.getLogger(Elevator.class);
    private static final Elevator INSTANCE = new Elevator();
    private ElevatorStates elevatorState = ElevatorStates.LOW;
    private double targetPosition;
    private final MotionMagicVoltage positionControl = new MotionMagicVoltage(0.0).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);
    TalonFX elevatorRight;
    TalonFX elevatorLeft;
    private final StatusSignal<Double> elevatorPosition, elevatorVelocity, elevatorCurrent;


    private Elevator() {
        RobustConfigurator configurator = new RobustConfigurator(logger);

        elevatorRight = new TalonFX(RobotMap.ELEVATOR_RIGHT_MOTOR);
        elevatorLeft = new TalonFX(RobotMap.ELEVATOR_LEFT_MOTOR);

        configurator.configureTalonFX(elevatorRight, elevatorConfig, "Right elevator motor");
        configurator.configureTalonFX(elevatorLeft, followerConfig, "Left elevator motor");

        elevatorPosition = elevatorRight.getPosition();
        elevatorVelocity = elevatorRight.getVelocity();
        elevatorCurrent = elevatorRight.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(RobotConfiguration.MECHANISM_UPDATE_FREQUENCY, elevatorCurrent, elevatorPosition, elevatorVelocity);

        ParentDevice.optimizeBusUtilizationForAll(elevatorRight, elevatorLeft);

    }

    public static Elevator getInstance() {
        return INSTANCE;
    }

    public enum ElevatorStates {
        HIGH,
        MID,
        LOW
    }

    public double getElevatorPosition() {
        return BaseStatusSignal.getLatencyCompensatedValue(elevatorPosition, elevatorVelocity);
    }

    public double getElevatorVelocity() {
        return elevatorVelocity.getValue();
    }

    public double getElevatorCurrent() {
        return elevatorCurrent.getValueAsDouble();
    }

    public ElevatorStates getElevatorState() {
        return elevatorState;
    }

    public void setElevatorPosition(double position) {
        targetPosition = MathUtil.clamp(position, ELEVATOR_MIN_POSE, ELEVATOR_MAX_POSE);
        elevatorRight.setControl(positionControl.withPosition(targetPosition));
    }

    public void setElevatorState(ElevatorStates state) {
        if (state != elevatorState) logger.info("Arm State Set To: " + state.name());
        this.elevatorState = state;

        switch (state) {
            case LOW -> setElevatorPosition(ELEVATOR_LOW_POSE);
            case MID -> setElevatorPosition(ELEVATOR_MID_POSE);
            case HIGH -> setElevatorPosition(ELEVATOR_HIGH_POSE);
        }
    }

    public void zero() {
        boolean isDisabled = RobotState.isDisabled();

        if (RobustConfigurator.retryConfigurator(() -> elevatorRight.setPosition(ELEVATOR_LOW_POSE),
                "Zeroed Elevator",
                "Failed to zero elevator",
                "Retrying elevator zeroing").isError() && isDisabled) {
            throw new RuntimeException("Uh...it didnt zero...I think you should power cycle");
        }

    }

    public void stop() {
        elevatorRight.stopMotor();
    }

    @Override
    public SubsystemIF initialize() {


        Commands.waitUntil(RobotState::isEnabled)
                .andThen(new ElevatorZeroCommand())
                .andThen(Commands.runOnce(() -> {
                    setElevatorPosition(ELEVATOR_LOW_POSE);
                }))
                .ignoringDisable(true).schedule();

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
        BaseStatusSignal.refreshAll(elevatorPosition, elevatorVelocity, elevatorCurrent);
    }


}