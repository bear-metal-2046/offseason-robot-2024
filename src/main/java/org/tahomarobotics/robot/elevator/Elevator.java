package org.tahomarobotics.robot.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.RobotMap;

import static org.tahomarobotics.robot.elevator.ElevatorConstants.*;

public class Elevator {

    public static final Logger logger = LoggerFactory.getLogger(Elevator.class);

    private static final Elevator INSTANCE = new Elevator();

    private ElevatorStates elevatorState = ElevatorStates.LOW;

    private double targetPosition;

    private final MotionMagicVoltage positionControl = new MotionMagicVoltage(0.0).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);

    TalonFX elevatorRight;
    TalonFX elevatorLeft;

    public ElevatorStates states;

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

    public void setElevatorPosition(double position) {
        targetPosition = MathUtil.clamp(position, ELEVATOR_MIN_POSE, ELEVATOR_MAX_POSE);
        elevatorRight.setControl(positionControl.withPosition(targetPosition));
    }

    public double getElevatorVelocity() {
        return elevatorVelocity.getValue();
    }

    public double getElevatorCurrent() {
        return elevatorCurrent.getValue();
    }

    public ElevatorStates getElevatorState() {
        return elevatorState;
    }

    public void setElevatorState(ElevatorStates state) {
        if (state != elevatorState) logger.info("Arm State Set To: " + state.name());
        this.elevatorState = state;

        switch (state){
            case LOW -> setElevatorPosition(ELEVATOR_LOW_POSE);
            case MID -> setElevatorPosition(ELEVATOR_MID_POSE);
            case HIGH -> setElevatorPosition(ELEVATOR_HIGH_POSE);
        }




    }


}