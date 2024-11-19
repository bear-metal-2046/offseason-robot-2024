package org.tahomarobotics.robot.mechanism;

import org.tahomarobotics.robot.util.SubsystemIF;

public class Mechanism extends SubsystemIF {
    private static final Mechanism INSTANCE = new Mechanism();

    public static Mechanism getInstance() {
        return INSTANCE;
    }

    private final MechanismMotor motor1;
    private final MechanismMotor motor2;

    private Mechanism() {
        motor1 = new MechanismMotor(1);
        motor2 = new MechanismMotor(2);
    }

    public void moveMotor1ToPosition(double pos) {
        motor1.moveToPosition(pos);
    }

    public void moveMotor2ToPosition(double pos) {
        motor2.moveToPosition(pos);
    }

    public void moveMotor1AtVelocity(double speed) {
        motor1.moveVelocity(speed);
    }

    public void moveMotor2AtVelocity(double speed) {
        motor2.moveVelocity(speed);
    }

    public double getMotor1Position() {
        return motor1.getPosition();
    }

    public double getMotor2Position() {
        return motor2.getPosition();
    }

    public double getMotor1Velocity() {
        return motor1.getVelocity();
    }

    public double getMotor2Velocity() {
        return motor2.getVelocity();
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
