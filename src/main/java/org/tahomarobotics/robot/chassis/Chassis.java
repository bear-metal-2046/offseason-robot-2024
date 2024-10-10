package org.tahomarobotics.robot.chassis;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.CalibrationData;

import java.io.Serializable;
import java.util.List;

public class Chassis {
    private static final Chassis INSTANCE = new Chassis();

    private final List<SwerveModule> modules;
    private final Gyro pigeon = new Gyro();

    private ChassisSpeeds targetSpeeds = new ChassisSpeeds();
    private final CalibrationData<Double[]> swerveCalibration;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Field2d fieldPose = new Field2d();
    private final Thread odometryThread;
    private boolean isFieldCentric = true;

    private Chassis() {
        // Read calibration from rio
        swerveCalibration = new CalibrationData<>("SwerveCalibration", new Double[]{0d, 0d, 0d, 0d});

        // Use calibration to make modules
        Double[] angularOffsets = swerveCalibration.get();
        modules = List.of(
                new SwerveModule(RobotMap.FRONT_LEFT_MOD, angularOffsets[0]),
                new SwerveModule(RobotMap.FRONT_RIGHT_MOD, angularOffsets[1]),
                new SwerveModule(RobotMap.BACK_LEFT_MOD, angularOffsets[2]),
                new SwerveModule(RobotMap.BACK_RIGHT_MOD, angularOffsets[3])
        );
    }
}
