package org.tahomarobotics.robot.elevator.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.tahomarobotics.robot.elevator.Elevator;
import org.tahomarobotics.robot.elevator.ElevatorConstants;

public class ElevatorZeroCommand extends Command {
    private final Elevator elevator = Elevator.getInstance();

    private final Timer timer = new Timer();

    public ElevatorZeroCommand() {
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        timer.restart();
        elevator.setVoltage(ElevatorConstants.ZEROING_VOLTAGE);
    }

    @Override
    public boolean isFinished() {
        return !elevator.isMoving() && timer.hasElapsed(0.1);
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stop();
        elevator.zero();
        elevator.setElevatorHeight(ElevatorConstants.ELEVATOR_MIN_POSE);
    }
}
