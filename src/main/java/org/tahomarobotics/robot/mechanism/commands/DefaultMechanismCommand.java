package org.tahomarobotics.robot.mechanism.commands;


import edu.wpi.first.wpilibj2.command.Command;
import org.tahomarobotics.robot.mechanism.Mechanism;

import java.util.function.DoubleSupplier;

public class DefaultMechanismCommand extends Command {
    private final DoubleSupplier motor1Speed;
    private final DoubleSupplier motor2Speed;

    public DefaultMechanismCommand(DoubleSupplier motor1Speed, DoubleSupplier motor2Speed) {
        this.motor1Speed = motor1Speed;
        this.motor2Speed = motor2Speed;

        addRequirements(Mechanism.getInstance());
    }

    @Override
    public void execute() {
        Mechanism.getInstance().moveMotor1AtPercent(motor1Speed.getAsDouble());
        Mechanism.getInstance().moveMotor2AtPercent(motor2Speed.getAsDouble());
    }
}
