package org.team1540.robot2019.commands.drivetrain;

import edu.wpi.first.wpilibj.command.PIDCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.apache.log4j.Logger;
import org.team1540.robot2019.Hardware;
import org.team1540.robot2019.OI;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.datastructures.twod.Twist2D;
import org.team1540.robot2019.datastructures.utils.TrigUtils;
import org.team1540.robot2019.utils.ControlUtils;
import org.team1540.robot2019.utils.TankDriveTwist2DInput;
import org.team1540.rooster.drive.pipeline.FeedForwardProcessor;
import org.team1540.rooster.drive.pipeline.UnitScaler;
import org.team1540.rooster.functional.Executable;

public class PointDrive extends PIDCommand {

    public static final Logger logger = Logger.getLogger(PointDrive.class);

    private static double DEADZONE;// = 0;

    // Max/Min angular velocity
    private static double MIN_VEL_THETA;// = 0.4;
    private static double MAX_VEL_THETA;// = 5;
    private static double OUTPUT_SCALAR;// = 5;

    // Constants for angular VPID controller
//    private static final double ANGULAR_KP = -0.7;
//    private static final double ANGULAR_KI = 0;
//    private static final double ANGULAR_KD = -2;

    private static Double initAngleOffset;
    private static Double lastGoalAngle = null;

    private final TankDriveTwist2DInput twist2DInput;
    private final Executable pipeline;

    public PointDrive() {
        super(
            SmartDashboard.getNumber("PointDrive/P", 0),
            SmartDashboard.getNumber("PointDrive/I", 0),
            SmartDashboard.getNumber("PointDrive/D", 0)
        );
        MIN_VEL_THETA = SmartDashboard.getNumber("PointDrive/min", 0);
        MAX_VEL_THETA = SmartDashboard.getNumber("PointDrive/max", 0);
        OUTPUT_SCALAR = SmartDashboard.getNumber("PointDrive/outScalar", 0);
        DEADZONE = SmartDashboard.getNumber("PointDrive/deadZone", 0);

        System.out.println("Point drive init");
        PointDrive.initAngleOffset = Hardware.navx.getYawRadians();
        requires(Robot.drivetrain);
        twist2DInput = new TankDriveTwist2DInput(Tuning.drivetrainRadiusMeters);
        pipeline = twist2DInput
            .then(new FeedForwardProcessor(Tuning.driveKV, Tuning.driveVIntercept, 0))
            .then(new UnitScaler(Tuning.drivetrainTicksPerMeter, 10))
            .then(Robot.drivetrain.getPipelineOutput(false));
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
        if (OI.getPointDriveMagnitude() > 0.5) {
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
        if (lastGoalAngle == null || Math.abs(output) < DEADZONE) {
            cmdVelTheta = 0;
        }
        SmartDashboard.putNumber("PointDrive/Debug/cmdVelTheta", cmdVelTheta);
        Twist2D cmdVel = new Twist2D(OI.getTankdriveLeftAxis() * -3, 0, -cmdVelTheta);
        twist2DInput.setTwist(cmdVel);
        pipeline.execute();
    }

    private double getAngleError(double x) {
        return TrigUtils.signedAngleError(x, Hardware.navx.getYawRadians());
    }
}
