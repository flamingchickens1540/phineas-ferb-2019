package org.team1540.robot2019.commands.drivetrain;

import org.apache.log4j.Logger;
import org.team1540.robot2019.Hardware;
import org.team1540.robot2019.OI;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.commands.auto.PointManualDriveCommand;
import org.team1540.robot2019.datastructures.utils.TrigUtils;

public class PointDrive extends PointManualDriveCommand {

    public static final Logger logger = Logger.getLogger(PointDrive.class);

    private static double OUTPUT_SCALAR = 20;

    // Max/Min angular velocity
    private static double MIN = 0;
    private static double MAX = 10;
    private static double DEADZONE = 0.05;

    // Constants for angular PID controller
    private static double P = 0.2;
    private static double I = 0;
    private static double D = 0.5;

    private static final double POINT_JOYSTICK_DEADZONE = 0.5;

    private static double THROTTLE_CONSTANT = 3; // Throttle constant for linear velocity

    private static double initAngleOffset = Hardware.navx.getYawRadians();
    private static double goalAngle = 0;

    public PointDrive() {
        super(P, I, D, OUTPUT_SCALAR, MAX, MIN, DEADZONE, THROTTLE_CONSTANT);
        requires(Robot.drivetrain);
    }

    @Override
    protected void initialize() {
        setGoalToCurrentAngle();
        logger.debug(String.format("Initialized with P:%f I:%f D:%f Max:%f Min:%f Deadzone:%f", P, I, D, MAX, MIN, DEADZONE));
    }

    private static void setGoalToCurrentAngle() {
        goalAngle = Hardware.navx.getYawRadians() - initAngleOffset;
    }

    public static void setInitAngleOffset(Double initAngleOffset) {
        PointDrive.initAngleOffset = initAngleOffset;
        setGoalToCurrentAngle();
    }

    @Override
    protected double returnAngleError() {
        if (OI.getPointDriveMagnitude() > POINT_JOYSTICK_DEADZONE) {
            goalAngle = OI.getPointDriveAngle();
        }
        return TrigUtils.signedAngleError(Hardware.navx.getYawRadians(), goalAngle + initAngleOffset);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
