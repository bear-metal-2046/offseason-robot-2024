package org.tahomarobotics.robot;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.chassis.commands.TeleopDriveCommand;
import org.tahomarobotics.robot.util.SubsystemIF;

public class OI extends SubsystemIF {
    private static final OI INSTANCE = new OI();

    private static final double ROTATIONAL_CURVE = 2.0;
    private static final double LINEAR_SENSITIVITY = 1.3;
    private static final double DEAD_ZONE = 0.09;

    public static OI getInstance() {
        return INSTANCE;
    }

    private final CommandXboxController driveController = new CommandXboxController(0);
    private final CommandXboxController manipController = new CommandXboxController(1);

    public OI() {
        CommandScheduler.getInstance().unregisterSubsystem(this);
        setDriveControls();
        setDefaultCommands();
    }

    public void setDriveControls() {

    }

    public void setDefaultCommands() {
        Chassis.getInstance().setDefaultCommand(new TeleopDriveCommand(
                () -> -desensitizePowerBased(driveController.getLeftY(), LINEAR_SENSITIVITY),
                () -> -desensitizePowerBased(driveController.getLeftX(), LINEAR_SENSITIVITY),
                () -> -desensitizePowerBased(driveController.getRightX(), ROTATIONAL_CURVE)
        ));
    }

    public double desensitizePowerBased(double value, double power) {
        value = deadBand(value, DEAD_ZONE);
        value *= Math.pow(Math.abs(value), power - 1);
        return value;
    }

    public double deadBand(double value, double deadZone) {
        if (Math.abs(value) > deadZone) {
            if (value > 0.0) {
                return (value - deadZone) / (1.0 - deadZone);
            } else {
                return (value + deadZone) / (1.0 - deadZone);
            }
        } else {
            return 0.0;
        }
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
