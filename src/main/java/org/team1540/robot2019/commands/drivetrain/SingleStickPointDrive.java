package org.team1540.robot2019.commands.drivetrain;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.PIDCommand;
import org.apache.log4j.Logger;
import org.team1540.robot2019.Hardware;
import org.team1540.robot2019.OI;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.datastructures.twod.Twist2D;
import org.team1540.robot2019.datastructures.utils.TrigUtils;
import org.team1540.robot2019.utils.ControlUtils;
import org.team1540.rooster.Utilities;

public class SingleStickPointDrive extends PIDCommand {

    public static final Logger logger = Logger.getLogger(SingleStickPointDrive.class);

    // Max/Min angular velocity
    private static final double MIN_VEL_THETA = 0.4;
    private static final double MAX_VEL_THETA = 5;

    private static final double OUTPUT_SCALAR = 5;

    // Constants for angular VPID controller
    private static final double ANGULAR_KP = -0.7;
    private static final double ANGULAR_KI = 0;
    private static final double ANGULAR_KD = -2;

    private static final double FINE_ADJUST_SCALAR = 2;

    private static Double initAngleOffset;
    private static Double goalAngle = null;

    private boolean backwardsDrive = false;
    private boolean wasDeadzoned = false;
    private boolean pointStick = false;

    public SingleStickPointDrive() {
        super(ANGULAR_KP, ANGULAR_KI, ANGULAR_KD);
        SingleStickPointDrive.initAngleOffset = Hardware.navx.getYawRadians();
        requires(Robot.drivetrain);
    }

    @Override
    protected void initialize() {
    }

    @Override
    protected void execute() {
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    public static void setInitAngleOffset(Double initAngleOffset) {
        goalAngle = null;
        SingleStickPointDrive.initAngleOffset = initAngleOffset;
    }

    private double lastTime = 0;

    @Override
    protected double returnPIDInput() {
        if (OI.getPointDriveMagnitude(Hand.kLeft) > Tuning.driveDeadzone) {
            goalAngle = OI.getPointDriveAngle(Hand.kLeft);
            pointStick = true;
        } else {
            pointStick = false;
            if (OI.getPointDriveMagnitude(Hand.kRight) > Tuning.driveDeadzone) {
                goalAngle = OI.getPointDriveAngle(Hand.kRight);
                double currentAngle = Hardware.navx.getYawRadians();
                double newStickError = TrigUtils.signedAngleError(goalAngle, currentAngle);
                if (wasDeadzoned && Math.abs(newStickError) > Math.PI / 2) {
                    backwardsDrive = !backwardsDrive;
                }
                wasDeadzoned = false;
            } else {
                wasDeadzoned = true;
                double fineAdjust = OI.getPointDriveFineLeft() - OI.getPointDriveFineRight();
                double currentTime = Timer.getFPGATimestamp();
                double deltaTime = currentTime - lastTime;
                if (deltaTime >= 1) {
                    goalAngle += fineAdjust * deltaTime * FINE_ADJUST_SCALAR;
                    lastTime = currentTime;
                }
            }
        }
        if (goalAngle == null) {
            return 0;
        }
        return getAngleError((backwardsDrive ? Math.PI : 0) + goalAngle + initAngleOffset);
    }

    @Override
    protected void usePIDOutput(double output) {
        output *= OUTPUT_SCALAR;
        double cmdVelTheta = ControlUtils.velocityPosNegConstrain(output, MAX_VEL_THETA, MIN_VEL_THETA);
        if (goalAngle == null || Math.abs(output) < 0.01) {
            cmdVelTheta = 0;
        }
        double xVel = Utilities.scale(OI.getPointDriveMagnitude(Hand.kRight), 2) * 0.7 * (backwardsDrive ? -1 : 1);
        if (pointStick) {
            xVel = 0;
        }
        Twist2D cmdVel = new Twist2D(xVel, 0, cmdVelTheta * 0.1); // TODO: Remove temporary 0.1 constant and re-tune
        Robot.drivetrain.setPercentTwist(cmdVel);
    }

    private double getAngleError(double x) {
        return TrigUtils.signedAngleError(x, Hardware.navx.getYawRadians());
    }
}
