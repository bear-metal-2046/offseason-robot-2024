package org.tahomarobotics.robot;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.tahomarobotics.robot.elevator.Elevator;
import org.tahomarobotics.robot.elevator.ElevatorConstants;
import org.tahomarobotics.robot.elevator.commands.ElevatorDefaultCommand;
import org.tahomarobotics.robot.elevator.commands.ElevatorMoveCommand;
import org.tahomarobotics.robot.util.SubsystemIF;

public class OI extends SubsystemIF {

    private final static OI INSTANCE = new OI();

    private final CommandXboxController manipController = new CommandXboxController(1);

    public OI getInstance(){
        return INSTANCE;
    }
    private void configureBindings(){
        Elevator elevator = Elevator.getInstance();

        manipController.povUp().onTrue(Commands.runOnce(elevator::toHigh).andThen(new ElevatorMoveCommand()));
        manipController.povRight().onTrue(Commands.runOnce(elevator::toMid).andThen(new ElevatorMoveCommand()));
        manipController.povDown().onTrue(Commands.runOnce(elevator::toLow).andThen(new ElevatorMoveCommand()));

    }

    private void setDefaultCommands(){
        Elevator.getInstance().setDefaultCommand(new ElevatorDefaultCommand(manipController::getLeftY));
    }

    public OI (){
        CommandScheduler.getInstance().unregisterSubsystem(this);

        configureBindings();
        setDefaultCommands();
    }

    @Override
    public double getEnergyUsed() {
        return 0;
    }

    @Override
    public double getTotalCurrent() {
        return 0;
    }
}
