package org.tahomarobotics.robot.mechanism;

import com.ctre.phoenix6.hardware.TalonFX;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.SubsystemIF;

public class Mechanism extends SubsystemIF {
    private static final Mechanism INSTANCE = new Mechanism();

    public static Mechanism getInstance() {
        return INSTANCE;
    }

    private final TalonFX motor1 = new TalonFX(RobotMap.MECHANISM_MOTOR_1);
    private final TalonFX motor2 = new TalonFX(RobotMap.MECHANISM_MOTOR_2);

    private Mechanism() {
        motor1.getConfigurator().apply(MechanismConstants.mechanismMotorConfig);
        motor2.getConfigurator().apply(MechanismConstants.mechanismMotorConfig);
    }

    public void moveMotor1AtPercent(double percent) {
        motor1.set(percent * MechanismConstants.MULTIPLIER);
    }

    public void moveMotor2AtPercent(double percent) {
        motor2.set(percent * MechanismConstants.MULTIPLIER);
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
