package org.tahomarobotics.robot.elevator.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.tahomarobotics.robot.elevator.Elevator;
import org.tahomarobotics.robot.elevator.ElevatorConstants;

import java.util.function.DoubleSupplier;

public class ElevatorDefaultCommand extends Command {
    private final Elevator elevator = Elevator.getInstance();
    private final DoubleSupplier input;

    public ElevatorDefaultCommand(DoubleSupplier input) {
        this.input = input;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.setVelocity(-input.getAsDouble() * ElevatorConstants.ELEVATOR_MAX_VELOCITY);
    }
}
