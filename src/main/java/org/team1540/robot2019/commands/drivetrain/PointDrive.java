package org.team1540.robot2019.commands.drivetrain;

import edu.wpi.first.wpilibj.command.PIDCommand;
import org.apache.log4j.Logger;
import org.team1540.robot2019.Hardware;
import org.team1540.robot2019.OI;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.datastructures.twod.Twist2D;
import org.team1540.robot2019.datastructures.utils.TrigUtils;
import org.team1540.robot2019.utils.ControlUtils;

public class PointDrive extends PIDCommand {

    public static final Logger logger = Logger.getLogger(PointDrive.class);

    // Max/Min angular velocity
    private static final double MIN_VEL_THETA = 0.4;
    private static final double MAX_VEL_THETA = 5;

    private static final double OUTPUT_SCALAR = 5;

    // Constants for angular VPID controller
    private static final double ANGULAR_KP = -0.7;
    private static final double ANGULAR_KI = 0;
    private static final double ANGULAR_KD = -2;

    private static Double initAngleOffset;
    private static Double lastGoalAngle = null;

    public PointDrive() {
        super(ANGULAR_KP, ANGULAR_KI, ANGULAR_KD);
        PointDrive.initAngleOffset = Hardware.navx.getYawRadians();
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

    public static void manualResetGoal() {
        lastGoalAngle = Hardware.navx.getYawRadians() - initAngleOffset;
    }

    public static void setInitAngleOffset(Double initAngleOffset) {
        lastGoalAngle = null;
        PointDrive.initAngleOffset = initAngleOffset;
    }

    @Override
    protected double returnPIDInput() {
        if (OI.getPointDriveMagnatude() > 0.5) {
            lastGoalAngle = OI.getPointDriveAngle();
        }
        if (lastGoalAngle == null) {
            return 0;
        }
        return getAngleError(lastGoalAngle + initAngleOffset);
    }

    @Override
    protected void usePIDOutput(double output) {
        output *= OUTPUT_SCALAR;
        double cmdVelTheta = ControlUtils.velocityPosNegConstrain(output, MAX_VEL_THETA, MIN_VEL_THETA);
        if (lastGoalAngle == null || Math.abs(output) < 0.01) {
            cmdVelTheta = 0;
        }
        Twist2D cmdVel = new Twist2D(OI.getTankdriveLeftAxis() * -0.7, 0, cmdVelTheta * 0.1); // TODO: Remove temporary 0.1 constant and re-tune
        Robot.drivetrain.setPercentTwist(cmdVel);
    }

    private double getAngleError(double x) {
        return TrigUtils.signedAngleError(x, Hardware.navx.getYawRadians());
    }
}
