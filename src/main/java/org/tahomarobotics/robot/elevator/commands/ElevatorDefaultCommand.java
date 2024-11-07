package org.tahomarobotics.robot.elevator.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.tahomarobotics.robot.elevator.Elevator;
import org.tahomarobotics.robot.elevator.ElevatorConstants;

import java.util.function.DoubleSupplier;

public class ElevatorDefaultCommand extends Command {

    private final Elevator elevator = Elevator.getInstance();

    private final DoubleSupplier input;

    @Override
    public void execute() {
        // add get postion
        if (Math.abs(elevator.getElevatorVelocity()) <= ElevatorConstants.VELOCITY_ELIPSON || Math.abs(elevator.getElevatorPosition() - ElevatorConstants.ELEVATOR_MIN_POSE) <= ElevatorConstants.POSITION_ELIPSON || Math.abs(elevator.getElevatorPosition() - ElevatorConstants.ELEVATOR_MAX_POSE) <= ElevatorConstants.POSITION_ELIPSON) {
            elevator.stop();
        }else{
            elevator.move(input);
        }
    }


    public ElevatorDefaultCommand(DoubleSupplier input) {
        addRequirements(this.elevator);
        this.input = input;


    }


}
