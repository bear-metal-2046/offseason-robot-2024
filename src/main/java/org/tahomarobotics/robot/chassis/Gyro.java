package org.tahomarobotics.robot.chassis;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;

import java.util.List;

public class Gyro {
    protected final Pigeon2 pigeon = new Pigeon2(RobotMap.PIGEON);
    private final StatusSignal<Double> yaw = pigeon.getYaw();
    private final StatusSignal<Double> yawVelocity = pigeon.getAngularVelocityZWorld();

    public record ValidYaw(Rotation2d yaw, boolean valid) {}

    Gyro() {
        pigeon.getConfigurator().apply(new Pigeon2Configuration());

        zeroHeading();

        yaw.setUpdateFrequency(RobotConfiguration.ODOMETRY_UPDATE_FREQUENCY);
        yawVelocity.setUpdateFrequency(RobotConfiguration.ODOMETRY_UPDATE_FREQUENCY);

        pigeon.optimizeBusUtilization();
    }

    public void zeroHeading() {
        pigeon.setYaw(0);
    }

    ValidYaw getYaw() {
        boolean valid = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
        return new ValidYaw(Rotation2d.fromDegrees(BaseStatusSignal.getLatencyCompensatedValue(yaw, yawVelocity)), valid);
    }

    List<BaseStatusSignal> getStatusSignals() {
        return List.of(yaw, yawVelocity);
    }
}
