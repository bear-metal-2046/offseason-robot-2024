package org.tahomarobotics.robot.chassis;

import com.ctre.phoenix6.BaseStatusSignal;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.Robot;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.chassis.commands.AlignSwerveCommand;
import org.tahomarobotics.robot.util.CalibrationData;
import org.tahomarobotics.robot.util.SubsystemIF;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Chassis extends SubsystemIF {
    private static final Logger logger = LoggerFactory.getLogger(Chassis.class);

    private static final Chassis INSTANCE = new Chassis();

    private final List<SwerveModule> modules;
    private final Gyro pigeon = new Gyro();

    private ChassisSpeeds targetSpeeds = new ChassisSpeeds();
    private ChassisSpeeds currentSpeeds = new ChassisSpeeds();
    private ChassisSpeeds currentAcceleration = new ChassisSpeeds();
    private final LinearFilter
            xAccelFilter = LinearFilter.movingAverage(3),
            yAccelFilter = LinearFilter.movingAverage(3),
            rotAccelFilter = LinearFilter.movingAverage(3);

    private final CalibrationData<Double[]> swerveCalibration;
    private final SwerveDriveKinematics kinematics;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Field2d fieldPose = new Field2d();
    private final Thread odometryThread;
    private boolean isFieldCentric = true;
    private Rotation2d heading = new Rotation2d();
    private SwerveModulePosition[] lastModulePosition;

    private final SwerveDriveLimiter accelerationLimiter;

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

        accelerationLimiter = new SwerveDriveLimiter(getSwerveModuleStates(), ChassisConstants.ACCELERATION_LIMIT);

        odometryThread = new Thread(this::odometryThread);
        odometryThread.start();
    }

    public static Chassis getInstance() {
        return INSTANCE;
    }

    @Override
    public SubsystemIF initialize() {
        SmartDashboard.putData("AlignSwerve", new AlignSwerveCommand());
        pigeon.zeroHeading();

        var gyro = getYaw();
        var modules = getSwerveModulePositions();
        synchronized (poseEstimator) {
            //changed SwerveDriveWheelPositions to passing in an array of the module positions
            //because SwerveDriveWheelPositions was just deleted in 2025.1.1 beta
            //if that broke something blame me :(
            poseEstimator.resetPosition(gyro, modules, new Pose2d());
        }

        return this;
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

    private SwerveModulePosition[] calculateModuleDeltas(SwerveModulePosition[] last, SwerveModulePosition[] current) {
        SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[current.length];
        for (int i = 0; i < current.length; i++) {
            moduleDeltas[i] = new SwerveModulePosition(current[i].distanceMeters - last[i].distanceMeters, current[i].angle);
        }

        return moduleDeltas;
    }

    // CALIBRATION
    public void initializeCalibration() {
        modules.forEach(SwerveModule::initCalibration);
    }

    public void finalizeCalibration() {
        swerveCalibration.set(
                modules.stream()
                        .map(SwerveModule::finalizeCalibration)
                        .toArray(Double[]::new)
        );
    }

    public void cancelCalibration() {
        modules.forEach(SwerveModule::cancelCalibration);
    }

    // GETTERS

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        return modules.stream().map(SwerveModule::getPosition).toArray(SwerveModulePosition[]::new);
    }

    private SwerveModuleState[] getSwerveModuleStates() {
        return modules.stream().map(SwerveModule::getState).toArray(SwerveModuleState[]::new);
    }

    public Rotation2d getYaw() {
        return heading;
    }

    // DRIVING

    public void drive(ChassisSpeeds velocity, boolean isFieldCentric) {
        if (isFieldCentric) {
            velocity = ChassisSpeeds.fromFieldRelativeSpeeds(velocity, getPose().getRotation());
        }
        targetSpeeds = velocity;
    }

    public void drive(ChassisSpeeds velocity) {
        if (!isFieldCentric && DriverStation.getAlliance().orElse(null) == DriverStation.Alliance.Red) {
            velocity = new ChassisSpeeds(-velocity.vxMetersPerSecond, -velocity.vyMetersPerSecond, velocity.omegaRadiansPerSecond);
        }

        drive(velocity, isFieldCentric);
    }

    private void setSwerveStates(SwerveModuleState[] states) {
        for (int i = 0; i < states.length; i++) {
            modules.get(i).setDesiredState(states[i]);
        }
    }

    private void updatePosition() {
        SwerveModulePosition[] positions;

        synchronized (modules) {
            positions = getSwerveModulePositions();
        }

        synchronized (pigeon) {
            var validYaw = pigeon.getYaw();

            // If pigeon yaw is valid, accept it as the real value
            if (validYaw.valid()) {
                heading = validYaw.yaw();
            // Else, calculate yaw from odometry by getting position deltas
            } else {
                SwerveModulePosition[] deltas = calculateModuleDeltas(lastModulePosition, positions);
                Twist2d twist = kinematics.toTwist2d(deltas);
                heading = heading.plus(new Rotation2d(twist.dtheta));
            }
        }

        lastModulePosition = Arrays.copyOf(positions, positions.length);
        synchronized (poseEstimator) {
            poseEstimator.update(heading, positions);
        }
    }

    @Override
    public void periodic() {
        modules.forEach(SwerveModule::periodic);

        Pose2d pose = getPose();

        ChassisSpeeds newChassisSpeeds = kinematics.toChassisSpeeds(getSwerveModuleStates());
        ChassisSpeeds unfilteredAcceleration = newChassisSpeeds.minus(currentSpeeds);
        currentAcceleration = new ChassisSpeeds(
            xAccelFilter.calculate(ChassisConstants.clampAccel(unfilteredAcceleration.vxMetersPerSecond)),
            yAccelFilter.calculate(ChassisConstants.clampAccel(unfilteredAcceleration.vyMetersPerSecond)),
            rotAccelFilter.calculate(ChassisConstants.clampAccel(unfilteredAcceleration.omegaRadiansPerSecond))
        );
        currentSpeeds = newChassisSpeeds;

        fieldPose.setRobotPose(pose);
        SmartDashboard.putData(fieldPose);

        if (RobotState.isEnabled()) {
            var swerveModuleStates = kinematics.toSwerveModuleStates(targetSpeeds);
            SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, ChassisConstants.MAX_VELOCITY);
            swerveModuleStates = accelerationLimiter.calculate(swerveModuleStates);
            setSwerveStates(swerveModuleStates);
        }
    }

    @Override
    public double getEnergyUsed() {
        return 0;
    }

    @Override
    public double getTotalCurrent() {
        return 0;
    }
}
