package org.tahomarobotics.robot.chassis;

import edu.wpi.first.math.geometry.Translation2d;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.RobotMap;

public class SwerveModule {
    private static final Logger logger = LoggerFactory.getLogger(SwerveModule.class);

    public final String name;
    private final Translation2d translationOffset;
    private double angularOffset;
    public SwerveModule(RobotMap.SwerveModuleDescriptor descriptor, double angularOffset){

        name = descriptor.moduleName();
        translationOffset = descriptor.offset();
        this.angularOffset = angularOffset;
    }
}
