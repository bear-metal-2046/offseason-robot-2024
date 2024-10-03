package org.tahomarobotics.robot.chassis;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.RobotMap;

public class SwerveModuleIO {
    private static final org.slf4j.Logger logger = LoggerFactory.getLogger(SwerveModuleIO.class);
    protected String name;

//    private final TalonFX driveMotor;
//
//    private final TalonFX steerMotor;
//
//    private final CANcoder steerAbsEncoder;

    private double angularOffset;

    SwerveModuleIO(RobotMap.SwerveModuleDescriptor descriptor, double angularOffset) {
        name = descriptor.moduleName();
        this.angularOffset = angularOffset;

    }
}
