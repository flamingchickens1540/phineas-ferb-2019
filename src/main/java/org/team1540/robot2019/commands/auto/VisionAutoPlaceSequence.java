package org.team1540.robot2019.commands.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.apache.log4j.Logger;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.commands.hatch.PlaceHatchSequence;
import org.team1540.robot2019.datastructures.utils.RotationUtils;
import org.team1540.robot2019.datastructures.utils.TrigUtils;
import org.team1540.robot2019.utils.WaitUntilCommand;

public class VisionAutoPlaceSequence extends CommandGroup {

    private static final Logger logger = Logger.getLogger(VisionAutoPlaceSequence.class);

    public static double MAX_DISTANCE = 0.4;
    public static double MAX_ANGLE_ERROR_DEGREES = 3;
    public static double MAX_RELATIVE_ANGLE = 10;

    public VisionAutoPlaceSequence() {
        SmartDashboard.putNumber("VisionAutoPlaceSequence/MAX_DISTANCE", MAX_DISTANCE);
        SmartDashboard.putNumber("VisionAutoPlaceSequence/MAX_ANGLE_ERROR_DEGREES", MAX_ANGLE_ERROR_DEGREES);
        SmartDashboard.putNumber("VisionAutoPlaceSequence/MAX_RELATIVE_ANGLE", MAX_RELATIVE_ANGLE);

        addSequential(new WaitUntilCommand(() -> isDistanceReached() && isAngleReached() && isSmallRelativeAngle()));
        addSequential(new PlaceHatchSequence(false, true));
    }

    private boolean isDistanceReached() {
        return Robot.drivetrain.getDriveCommand().getLineupLocalization().getDistanceToVisionTarget() < MAX_DISTANCE;
    }

    private boolean isAngleReached() {
        return Math.abs(Robot.drivetrain.getDriveCommand().getLineupLocalization().returnAngleError(Math.PI)) < Math.toRadians(MAX_ANGLE_ERROR_DEGREES);
    }

    private boolean isSmallRelativeAngle() {
        double goalYaw = RotationUtils.getRPYVec(Robot.drivetrain.getDriveCommand().getLineupLocalization().getGoal().getOrientation()).getZ();
        double currentYaw = RotationUtils.getRPYVec(Robot.odometry.getOdomToBaseLink().getOrientation()).getZ();
        double relativeAngle = TrigUtils.signedAngleError(goalYaw, currentYaw);
        SmartDashboard.putNumber("VisionAutoPlaceSequence/relativeAngle", relativeAngle);
        return Math.abs(relativeAngle) < Math.toRadians(MAX_RELATIVE_ANGLE);
    }

    @Override
    protected void initialize() {
        MAX_DISTANCE = SmartDashboard.getNumber("VisionAutoPlaceSequence/MAX_DISTANCE", MAX_DISTANCE);
        MAX_ANGLE_ERROR_DEGREES = SmartDashboard.getNumber("VisionAutoPlaceSequence/MAX_ANGLE_ERROR_DEGREES", MAX_ANGLE_ERROR_DEGREES);
        logger.debug("Init");
    }

    @Override
    protected void execute() {

        // Debug TODO: Remove this
        boolean distanceReached = isDistanceReached();
        boolean angleReached = isAngleReached();
        boolean smallRelativeAngle = isSmallRelativeAngle();

        String reason;
        if (!distanceReached && !smallRelativeAngle) {
            reason = "Both";
        } else if (!distanceReached) {
            reason = "Distance";
        } else if (!angleReached) {
            reason = "Point Angle";
        } else if (!smallRelativeAngle) {
            reason = "Relative Angle";
        } else {
            reason = "None";
        }

        SmartDashboard.putString("VisionAutoPlaceSequence/waitingReason", reason);
    }

    @Override
    protected void end() {
        logger.debug("End");
        SmartDashboard.putString("VisionAutoPlaceSequence/waitingReason", "-");
    }

    @Override
    protected void interrupted() {
        end();
    }
}
