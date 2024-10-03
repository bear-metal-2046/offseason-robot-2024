package org.tahomarobotics.robot.chassis;

import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.CalibrationData;

import java.io.Serializable;
import java.util.List;

public class Chassis {
    private static final Chassis INSTANCE = new Chassis();

    private final List<SwerveModule> modules;
    private final CalibrationData<Double[]> swerveCalibration;

    private Chassis() {
        swerveCalibration = new CalibrationData<>("SwerveCalibration", new Double[]{0d,0d,0d,0d});

        Double[] angularOffsets = swerveCalibration.get();
        modules = List.of(
                new SwerveModule(RobotMap.FRONT_LEFT_MOD, angularOffsets[0] ),
                new SwerveModule(RobotMap.BACK_LEFT_MOD, angularOffsets[1]),
                new SwerveModule(RobotMap.BACK_LEFT_MOD, angularOffsets[2]),
                new SwerveModule(RobotMap.BACK_LEFT_MOD, angularOffsets[3])
        );
    }
}
