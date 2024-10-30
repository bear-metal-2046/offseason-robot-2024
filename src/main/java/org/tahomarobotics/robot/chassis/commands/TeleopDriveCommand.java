package org.tahomarobotics.robot.chassis.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.chassis.ChassisConstants;

import java.util.function.DoubleSupplier;

public class TeleopDriveCommand extends Command {
    private final Chassis chassis = Chassis.getInstance();
    private final ChassisSpeeds velocityInput = new ChassisSpeeds();

    private final DoubleSupplier fwd, str, rot;

    private final double maxVelocity;
    private final double maxRotationalVelocity;

    public TeleopDriveCommand(DoubleSupplier fwd, DoubleSupplier str, DoubleSupplier rot) {
        this.fwd = fwd;
        this.str = str;
        this.rot = rot;
        addRequirements(chassis);

        maxVelocity = ChassisConstants.MAX_VELOCITY;
        maxRotationalVelocity = ChassisConstants.MAX_ANGULAR_VELOCITY;
    }

    @Override
    public void execute() {
        velocityInput.vxMetersPerSecond = fwd.getAsDouble() * maxVelocity;
        velocityInput.vyMetersPerSecond = str.getAsDouble() * maxVelocity;
        velocityInput.omegaRadiansPerSecond = rot.getAsDouble() * maxVelocity;

        chassis.drive(velocityInput);
    }
}
