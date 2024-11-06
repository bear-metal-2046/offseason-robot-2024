package org.tahomarobotics.robot.chassis.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.chassis.ChassisConstants;

import java.util.function.DoubleSupplier;

public class TeleopDriveCommand extends Command {
    private final Chassis chassis = Chassis.getInstance();
    private final ChassisSpeeds velocityInput = new ChassisSpeeds();

    private final DoubleSupplier x, y, omega;

    private final double maxVelocity;
    private final double maxRotationalVelocity;

    public TeleopDriveCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega) {
        this.x = x;
        this.y = y;
        this.omega = omega;
        addRequirements(chassis);

        maxVelocity = ChassisConstants.MAX_VELOCITY;
        maxRotationalVelocity = ChassisConstants.MAX_ANGULAR_VELOCITY;
    }

    @Override
    public void execute() {
        velocityInput.vxMetersPerSecond = x.getAsDouble() * maxVelocity;
        velocityInput.vyMetersPerSecond = y.getAsDouble() * maxVelocity;
        velocityInput.omegaRadiansPerSecond = omega.getAsDouble() * maxVelocity;

        chassis.drive(velocityInput);
    }
}
