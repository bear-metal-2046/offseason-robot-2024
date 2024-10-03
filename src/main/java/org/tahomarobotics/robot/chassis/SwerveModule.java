package org.tahomarobotics.robot.chassis;

import edu.wpi.first.math.geometry.Translation2d;
import org.tahomarobotics.robot.RobotMap;

public class SwerveModule {

    SwerveModuleIO io;
    public final String name;
    private final Translation2d translationOffset;
    public SwerveModule(RobotMap.SwerveModuleDescriptor descriptor, double angularOffset){
        io = new SwerveModuleIO(descriptor, angularOffset);
        name = descriptor.moduleName();
        translationOffset = descriptor.offset();
    }
}
