package org.tahomarobotics.robot.elevator.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.tahomarobotics.robot.elevator.Elevator;

import java.util.function.DoubleSupplier;

public class ElevatorDefaultCommand extends Command {

    private final Elevator elevator = Elevator.getInstance();

    private final DoubleSupplier input;

    public ElevatorDefaultCommand(DoubleSupplier input) {
        addRequirements(elevator);
        this.input = input;
    }
    @Override
    public void execute() {
        if (elevator.nearBounds()) {
            elevator.stop();
        }else{
            elevator.setVoltage(input.getAsDouble());
        }
    }

}
