package org.tahomarobotics.robot.chassis;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.RobotMap;

import java.util.List;

public class SwerveModule {
    private static final Logger logger = LoggerFactory.getLogger(SwerveModule.class);

    public final String name;
    private final Translation2d translationOffset;
    private double angularOffset;

    private final StatusSignal<Double> steerPosition;
    private final StatusSignal<Double> steerVelocity;
    private final StatusSignal<Double> drivePosition;
    private final StatusSignal<Double> driveVelocity;
    private final StatusSignal<Double> driveAcceleration;
    private final StatusSignal<Double> driveCurrent;
    private final StatusSignal<Double> steerCurrent;

    public SwerveModule(RobotMap.SwerveModuleDescriptor descriptor, double angularOffset){
        name = descriptor.moduleName();
        translationOffset = descriptor.offset();
        this.angularOffset = angularOffset;
    }

    public Translation2d getTranslationOffset() {
        return translationOffset;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), Rotation2d.fromRotations(getSteerAngle()));
    }

    private double getSteerAngle() {
        return 0;
    }

    private double getDrivePosition() {
        return 0;
    }

    public List<BaseStatusSignal> getStatusSignals() {
        return List.of(
                drivePosition,
                driveAcceleration,
                driveVelocity,
                steerPosition,
                steerVelocity,
                driveCurrent,
                steerCurrent
        );
    }
}
