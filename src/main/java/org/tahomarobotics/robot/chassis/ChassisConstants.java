package org.tahomarobotics.robot.chassis;

import edu.wpi.first.math.geometry.Translation2d;

public class ChassisConstants {

    public static final double TRACK_WIDTH = 0.5816;
    public static final double WHEELBASE = 0.8194;
    public static final double HALF_TRACK_WIDTH = TRACK_WIDTH / 2;
    public static final double HALF_WHEELBASE = WHEELBASE / 2;

    public static final Translation2d FRONT_LEFT_OFFSET = new Translation2d(HALF_WHEELBASE, HALF_TRACK_WIDTH);
    public static final Translation2d FRONT_RIGHT_OFFSET = new Translation2d(HALF_WHEELBASE, -HALF_TRACK_WIDTH);
    public static final Translation2d BACK_LEFT_OFFSET = new Translation2d(-HALF_WHEELBASE, HALF_TRACK_WIDTH);
    public static final Translation2d BACK_RIGHT_OFFSET = new Translation2d(-HALF_WHEELBASE, -HALF_TRACK_WIDTH);
}
