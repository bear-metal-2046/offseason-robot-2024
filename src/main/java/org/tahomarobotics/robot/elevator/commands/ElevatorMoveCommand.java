package org.tahomarobotics.robot.elevator.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.tahomarobotics.robot.elevator.Elevator;
import org.tahomarobotics.robot.elevator.ElevatorConstants;

import static org.tahomarobotics.robot.elevator.ElevatorConstants.*;

public class ElevatorMoveCommand extends Command {
    private final Elevator elevator = Elevator.getInstance();
    private final Timer timer = new Timer();

    private double targetPosition;


    public ElevatorMoveCommand() {
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        switch (elevator.elevatorState) {
            case LOW -> targetPosition = ELEVATOR_LOW_POSE;
            case MID -> targetPosition = ELEVATOR_MID_POSE;
            case HIGH -> targetPosition = ELEVATOR_HIGH_POSE;
        }
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        elevator.setElevatorPosition(targetPosition);
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        elevator.stop();
    }

    @Override
    public boolean isFinished() {
        return targetPosition - elevator.getElevatorPosition() <= ElevatorConstants.POSITION_ELIPSON || timer.hasElapsed(2.0);
    }
}
