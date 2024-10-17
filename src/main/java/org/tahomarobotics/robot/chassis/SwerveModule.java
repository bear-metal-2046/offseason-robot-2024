package org.tahomarobotics.robot.chassis;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;

import java.util.List;

public class SwerveModule {
    private static final Logger logger = LoggerFactory.getLogger(SwerveModule.class);

    public final String name;
    private final Translation2d translationOffset;
    private double angularOffset;

    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final CANcoder steerEncoder;

    private SwerveModuleState targetState = new SwerveModuleState();

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

        driveMotor = new TalonFX(descriptor.driveId(), RobotConfiguration.CANBUS_NAME);
        steerMotor = new TalonFX(descriptor.steerId(), RobotConfiguration.CANBUS_NAME);
        steerEncoder = new CANcoder(descriptor.encoderId(), RobotConfiguration.CANBUS_NAME);

        // TODO: configure stuff here

        drivePosition = driveMotor.getPosition();
        driveVelocity = driveMotor.getVelocity();
        driveAcceleration = driveMotor.getAcceleration();

        steerPosition = steerEncoder.getPosition();
        steerVelocity = steerEncoder.getVelocity();

        driveCurrent = driveMotor.getSupplyCurrent();
        steerCurrent = steerMotor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(RobotConfiguration.ODOMETRY_UPDATE_FREQUENCY,
                drivePosition,
                driveVelocity,
                driveAcceleration,
                steerVelocity,
                driveCurrent,
                steerCurrent
        );
        ParentDevice.optimizeBusUtilizationForAll(driveMotor, steerMotor, steerEncoder);
    }

    // Calibration methods

    public void initCalibration() {

    }

    public double finalizeCalibration() {

    }

    public void cancelCalibration() {

    }

    // Getters

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

    public void periodic() {

    }
}
