package org.team1540.robot2019.commands.drivetrain.pointdrive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.apache.log4j.Logger;
import org.team1540.robot2019.Hardware;
import org.team1540.robot2019.datastructures.utils.TrigUtils;

public class PointDriveAngleProvider implements PointAngleProvider {

    private static final Logger logger = Logger.getLogger(PointDriveAngleProvider.class);

    private static double OUTPUT_SCALAR = 20;

    // Max/Min angular velocity
    private static double MIN = 0;
    private static double MAX = 10;
    private static double DEADZONE = 0.05;

    // Constants for angular PID controller
    private static double P = 0.25;
    private static double I = 0;
    private static double D = 2;

    private static final double POINT_JOYSTICK_DEADZONE = 0.5;

    private static double THROTTLE_CONSTANT = 3; // Throttle constant for linear velocity

    private static double initAngleOffset = Hardware.navx.getYawRadians();
    private static double goalAngle = 0;

    public PointDriveAngleProvider() {
//        super(P, I, D, OUTPUT_SCALAR, MAX, MIN, DEADZONE, THROTTLE_CONSTANT);
//        requires(Robot.drivetrain);

        SmartDashboard.putNumber("PointDriveAngleProvider/ANGULAR_KP", P);
        SmartDashboard.putNumber("PointDriveAngleProvider/ANGULAR_KI", I);
        SmartDashboard.putNumber("PointDriveAngleProvider/ANGULAR_KD", D);
    }

    @Override
    public void initialize() {
        P = SmartDashboard.getNumber("PointDriveAngleProvider/ANGULAR_KP", P);
        I = SmartDashboard.getNumber("PointDriveAngleProvider/ANGULAR_KI", I);
        D = SmartDashboard.getNumber("PointDriveAngleProvider/ANGULAR_KD", D);

        setGoalToCurrentAngle();
        logger.debug(String.format("Initialized with P:%f I:%f D:%f Max:%f Min:%f Deadzone:%f", P, I, D, MAX, MIN, DEADZONE));
    }

    @Override
    public PointControlConfig getPointControlConfig() {
        return new PointControlConfig(OUTPUT_SCALAR, MIN, MAX, DEADZONE, P, I, D, THROTTLE_CONSTANT);
    }

    private static void setGoalToCurrentAngle() {
        goalAngle = Hardware.navx.getYawRadians() - initAngleOffset;
    }

    public static void setInitAngleOffset(Double initAngleOffset) {
        PointDriveAngleProvider.initAngleOffset = initAngleOffset;
        setGoalToCurrentAngle();
    }

    @Override
    public double returnAngleError(double defaultError) {
//        if (OI.getPointDriveMagnitude() > POINT_JOYSTICK_DEADZONE) {
//            goalAngle = OI.getPointDriveAngle();
//        }
        return TrigUtils.signedAngleError(goalAngle + initAngleOffset, Hardware.navx.getYawRadians());
    }
}
