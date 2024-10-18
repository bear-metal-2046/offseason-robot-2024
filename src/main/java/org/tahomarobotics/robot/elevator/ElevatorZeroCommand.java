package org.tahomarobotics.robot.elevator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;




public class ElevatorZeroCommand extends Command {

    private static final Logger logger = LoggerFactory.getLogger(ElevatorZeroCommand.class);

    private final Elevator elevator = Elevator.getInstance();

    Timer timer = new Timer();

    private boolean zeroed;

    @Override
    public void initialize(){
        logger.info("Zeroing Elevator");
    }

    @Override
    public void execute() {
        if (zeroed || Math.abs(elevator.getElevatorVelocity()) <
                ElevatorConstants.VELOCITY_ELIPSON && timer.hasElapsed(1.0)){
            elevator.stop();
            zeroed = true;
        }
    }

    @Override
    public boolean isFinished() {
        return zeroed;
    }

    @Override
    public void end(boolean interrupted) {
        elevator.zero();
        logger.info("Zeroed elevator");
    }
}
