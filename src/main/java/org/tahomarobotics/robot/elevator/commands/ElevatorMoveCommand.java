package org.tahomarobotics.robot.elevator.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.tahomarobotics.robot.elevator.Elevator;
import org.tahomarobotics.robot.elevator.ElevatorConstants;

import static org.tahomarobotics.robot.elevator.ElevatorConstants.*;

public class ElevatorMoveCommand extends Command {
    private final Elevator elevator = Elevator.getInstance();
    private final Timer timer = new Timer();

    private Elevator.ElevatorStates targetPosition;
    private final double targetPositionAsDouble;


    public ElevatorMoveCommand(Elevator.ElevatorStates targetPosition) {
        elevator.setElevatorState(targetPosition);
        targetPositionAsDouble = elevator.getTargetPosition();
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        elevator.setElevatorHeight(targetPositionAsDouble);
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        elevator.stop();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(targetPositionAsDouble - elevator.getElevatorHeight()) <= ElevatorConstants.POSITION_EPSILON || timer.hasElapsed(2.0);
    }
}
