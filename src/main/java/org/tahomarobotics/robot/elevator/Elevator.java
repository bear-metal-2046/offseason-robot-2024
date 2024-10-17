package org.tahomarobotics.robot.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.RobustConfigurator;
import static org.tahomarobotics.robot.elevator.ElevatorConstants.*;

public class Elevator {

    public static final Logger logger = LoggerFactory.getLogger(Elevator.class);

    private static final Elevator INSTANCE = new Elevator();

    TalonFX elevatorRight;
    TalonFX elevatorLeft;

    public ElevatorStates states;

    private final StatusSignal<Double> elevatorPosition, elevatorVelocity, elevatorCurrent;

    private Elevator() {
        RobustConfigurator configurator = new RobustConfigurator(logger);

        elevatorRight = new TalonFX(RobotMap.ELEVATOR_RIGHT_MOTOR);
        elevatorLeft = new TalonFX(RobotMap.ELEVATOR_LEFT_MOTOR);

        configurator.configureTalonFX(elevatorRight,elevatorConfig,"Right elevator motor");
        configurator.configureTalonFX(elevatorLeft,followerConfig,"Left elevator motor");

        elevatorPosition = elevatorRight.getPosition();
        elevatorVelocity = elevatorRight.getVelocity();
        elevatorCurrent = elevatorRight.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(100,elevatorCurrent,elevatorPosition,elevatorVelocity);

        ParentDevice.optimizeBusUtilizationForAll(elevatorRight, elevatorLeft);

    }

    public static Elevator getInstance() {return INSTANCE;}

    public enum ElevatorStates {
        HIGH,
        MID,
        LOW
    }

}
