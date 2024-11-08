package org.tahomarobotics.robot.elevator.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.elevator.Elevator;
import org.tahomarobotics.robot.elevator.ElevatorConstants;


public class ElevatorZeroCommand extends Command {

    private static final Logger logger = LoggerFactory.getLogger(ElevatorZeroCommand.class);

    private final Elevator elevator = Elevator.getInstance();

    Timer timer = new Timer();

    private boolean zeroed = false;

    public ElevatorZeroCommand() {addRequirements(this.elevator);}

    @Override
    public void initialize(){
        logger.info("Zeroing Elevator");
    }

    @Override
    public void execute() {

        elevator.stop();
    }

    @Override
    public boolean isFinished() {
        return !zeroed || Math.abs(elevator.getElevatorVelocity()) < ElevatorConstants.VELOCITY_EPSILON && timer.hasElapsed(5.0);
    }

    @Override
    public void end(boolean interrupted) {
        logger.info("Zeroed elevator");
    }
}
