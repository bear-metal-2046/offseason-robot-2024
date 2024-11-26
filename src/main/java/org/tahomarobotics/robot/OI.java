package org.tahomarobotics.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.chassis.commands.TeleopDriveCommand;
import org.tahomarobotics.robot.elevator.Elevator;
import org.tahomarobotics.robot.elevator.commands.ElevatorDefaultCommand;
import org.tahomarobotics.robot.elevator.commands.ElevatorMoveCommand;
import org.tahomarobotics.robot.mechanism.Mechanism;
import org.tahomarobotics.robot.mechanism.commands.DefaultMechanismCommand;
import org.tahomarobotics.robot.util.SubsystemIF;

public class OI extends SubsystemIF {
    private static final OI INSTANCE = new OI();

    private static final double ROTATIONAL_CURVE = 2.0;
    private static final double LINEAR_SENSITIVITY = 1.3;
    private static final double DEAD_ZONE = 0.09;

    private final Elevator elevator = Elevator.getInstance();
    private final Chassis chassis = Chassis.getInstance();
    private final Mechanism mechanism = Mechanism.getInstance();

    public static OI getInstance() {
        return INSTANCE;
    }

    private final CommandXboxController driveController = new CommandXboxController(0);
    private final CommandXboxController manipController = new CommandXboxController(1);

    public OI() {
        CommandScheduler.getInstance().unregisterSubsystem(this);
        configureBindings();
        setDefaultCommands();
    }

    public void configureBindings() {
        driveController.a().onTrue(Commands.runOnce(chassis::zeroHeading));

        manipController.povUp().onTrue(new ElevatorMoveCommand(Elevator.ElevatorStates.HIGH));
        manipController.povRight().onTrue(new ElevatorMoveCommand(Elevator.ElevatorStates.MID));
        manipController.povDown().onTrue(new ElevatorMoveCommand(Elevator.ElevatorStates.LOW));

        mechanism.setDefaultCommand(new DefaultMechanismCommand(
                () -> -deadBand(manipController.getRightY(), DEAD_ZONE),
                () -> deadBand(manipController.getLeftTriggerAxis() - manipController.getRightTriggerAxis(), DEAD_ZONE)
        ));

        SmartDashboard.putData("LOW", new ElevatorMoveCommand(Elevator.ElevatorStates.LOW));
        SmartDashboard.putData("MID", new ElevatorMoveCommand(Elevator.ElevatorStates.MID));
        SmartDashboard.putData("🚬", new ElevatorMoveCommand(Elevator.ElevatorStates.HIGH));
    }

    public void setDefaultCommands() {
        chassis.setDefaultCommand(new TeleopDriveCommand(
                () -> -desensitizePowerBased(driveController.getLeftY(), LINEAR_SENSITIVITY),
                () -> -desensitizePowerBased(driveController.getLeftX(), LINEAR_SENSITIVITY),
                () -> -desensitizePowerBased(driveController.getRightX(), ROTATIONAL_CURVE)
        ));

        elevator.setDefaultCommand(new ElevatorDefaultCommand(() -> deadBand(manipController.getLeftY(), DEAD_ZONE)));
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
