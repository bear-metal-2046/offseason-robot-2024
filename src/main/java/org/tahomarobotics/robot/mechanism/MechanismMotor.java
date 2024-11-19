package org.tahomarobotics.robot.mechanism;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class MechanismMotor {
    private final TalonFX motor;
    private final MotionMagicVoltage positionControl = new MotionMagicVoltage(0);
    private final MotionMagicVelocityVoltage velocityControl = new MotionMagicVelocityVoltage(0);

    public MechanismMotor(int motorID) {
        motor = new TalonFX(motorID);
        motor.getConfigurator().apply(MechanismConstants.mechanismMotorConfig);
    }

    // SETTERS

    public void moveToPosition(double pos) {
        motor.setControl(positionControl.withPosition(pos));
    }

    public void moveVelocity(double speed) {
        motor.setControl(velocityControl.withVelocity(speed * MechanismConstants.MAX_RPS));
    }

    // GETTERS

    public double getEnergyUsed() {
        return motor.getMotorVoltage().getValueAsDouble();
    }

    public double getTotalCurrent() {
        return motor.getStatorCurrent().getValueAsDouble();
    }

    public double getPosition() {
        return motor.getPosition().getValueAsDouble();
    }

    public double getVelocity() {
        return motor.getVelocity().getValueAsDouble();
    }
}
