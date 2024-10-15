package org.tahomarobotics.robot.chassis;

import com.ctre.phoenix6.BaseStatusSignal;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.struct.SwerveModuleStateStruct;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.CalibrationData;

import java.util.ArrayList;
import java.util.List;

public class Chassis {
    private static final Chassis INSTANCE = new Chassis();

    private final List<SwerveModule> modules;
    private final Gyro pigeon = new Gyro();

    private ChassisSpeeds targetSpeeds = new ChassisSpeeds();
    private final CalibrationData<Double[]> swerveCalibration;
    private final SwerveDriveKinematics kinematics;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Field2d fieldPose = new Field2d();
    private final Thread odometryThread;
    private boolean isFieldCentric = true;
    private SwerveModulePosition[] lastModulePosition;

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

        kinematics = new SwerveDriveKinematics(
                modules.stream()
                        .map(SwerveModule::getTranslationOffset)
                        .toArray(Translation2d[]::new)
        );

        lastModulePosition = getSwerveModulePositions();

        poseEstimator = new SwerveDrivePoseEstimator(
                kinematics,
                new Rotation2d(),
                getSwerveModulePositions(),
                new Pose2d(0.0, 0.0, new Rotation2d(0.0))
        );

        odometryThread = new Thread(this::odometryThread);
        odometryThread.start();
    }

    public static Chassis getInstance() {
        return INSTANCE;
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        return modules.stream().map(SwerveModule::getPosition).toArray(SwerveModulePosition[]::new);
    }

    private void odometryThread() {
        Threads.setCurrentThreadPriority(true, 1);

        // Get signals array
        List<BaseStatusSignal> signalList = new ArrayList<>(pigeon.getStatusSignals());
        for (var module : this.modules) {
            signalList.addAll(module.getStatusSignals());
        }

        BaseStatusSignal[] signals = signalList.toArray(BaseStatusSignal[]::new);


        while (true) {
            // Wait for all signals to arrive
            BaseStatusSignal.waitForAll(4 / RobotConfiguration.ODOMETRY_UPDATE_FREQUENCY, signals);
            updatePosition();
        }
    }

    private void updatePosition() {

    }
}
