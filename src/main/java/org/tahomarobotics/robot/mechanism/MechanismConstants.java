package org.tahomarobotics.robot.mechanism;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class MechanismConstants {
    public static final double MAX_RPS = 0;
    public static final TalonFXConfiguration mechanismMotorConfig = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                    .withKP(1)
                    .withKI(0)
                    .withKD(0)
            );
}
