package org.tahomarobotics.robot.mechanism;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class MechanismConstants {
    public static final double MULTIPLIER = 0.75;
    public static final TalonFXConfiguration mechanismMotorConfig = new TalonFXConfiguration().withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
}
