package org.team1540.robot2019;

import edu.wpi.first.wpilibj.SPI.Port;

public class RobotMap {

    // motors
    public static final int DRIVE_LEFT_A = 13;
    public static final int DRIVE_LEFT_B = 12;
    public static final int DRIVE_LEFT_C = 11;

    public static final int DRIVE_RIGHT_A = 1;
    public static final int DRIVE_RIGHT_B = 2;
    public static final int DRIVE_RIGHT_C = 3;

    public static final int ELEVATOR_L = 6;
    public static final int ELEVATOR_R = 7;

    public static final int WRIST_MOTOR = 8;

    public static final int CARGO_ROLLER_TOP = 9;
    public static final int CARGO_ROLLER_BOTTOM = 10;

    public static final int CLIMBER_ARM_L = 5;
    public static final int CLIMBER_ARM_R = 4;

    // pneumatics
    public static final int CLIMBER_CYLINDER_1 = 3;
    public static final int CLIMBER_CYLINDER_2 = 2;

    public static final int HATCH_SLIDE;
    public static final int HATCH_GRABBER;

    public static final int LED_GREEN = 5;
    public static final int LED_BLUE = 6;
    public static final int LED_RED = 4;

    // sensors and switches
    public static final int ELEVATOR_LIMIT_SENSOR = 1;

    public static final int WRIST_MID_SW;
    public static final int WRIST_BTM_SW;

    public static final int HATCH_EXTEND_SW = 4;

    public static final int CARGO_INTAKE_SENSOR = 0;

    public static final int PRESSURE_SENSOR = 3;


    // pdp ports
    public static final int PDP_DRIVE_LEFT_A = 0;
    public static final int PDP_DRIVE_LEFT_B = 1;
    public static final int PDP_DRIVE_LEFT_C = 2;
    public static final int PDP_DRIVE_RIGHT_A = 15;
    public static final int PDP_DRIVE_RIGHT_B = 14;
    public static final int PDP_DRIVE_RIGHT_C = 13;
    public static final int PDP_WRIST_MOTOR = 7;
    public static final int PDP_CARGO_MECH_TOP = 5;
    public static final int PDP_CARGO_MECH_BTM = 4;
    public static final int PDP_CLIMBER_ARM_L = 10;
    public static final int PDP_CLIMBER_ARM_R = 11;

    public static final Port NAVX = Port.kMXP;

    public static final double CAM_X = 0.086;
    public static final double CAM_Y = 0.099;
    public static final double CAM_Z = 1.12;
    public static final double CAM_PITCH;
    public static final double CAM_ROLL = Math.toRadians(0);
    public static final double CAM_YAW;

    public static final double HATCH_TARGET_HEIGHT = 0.74; // center
    public static final double ROCKET_BALL_TARGET_HEIGHT = 0.99; // center
//    public static final double HATCH_TARGET_HEIGHT = 0.652018; // bottom
//    public static final double ROCKET_BALL_TARGET_HEIGHT = 0.851408; // bottom

    static {
        if (Tuning.isComp) {
//            CAM_YAW = Math.toRadians(3);
//            CAM_PITCH = Math.toRadians(-30.0);
            CAM_YAW = Math.toRadians(4.01562054);
            CAM_PITCH = Math.toRadians(-27.8904396);
            HATCH_SLIDE = 7;
            HATCH_GRABBER = 1;//0;
            WRIST_MID_SW = 2;
            WRIST_BTM_SW = 3;
        } else {
            CAM_YAW = Math.toRadians(0.616);
            CAM_PITCH = Math.toRadians(-33.08);
            HATCH_SLIDE = 1;
            HATCH_GRABBER = 0;
            WRIST_MID_SW = 2;
            WRIST_BTM_SW = 3;
        }
    }
}
