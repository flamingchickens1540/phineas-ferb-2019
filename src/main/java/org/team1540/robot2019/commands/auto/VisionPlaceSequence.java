package org.team1540.robot2019.commands.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.apache.log4j.Logger;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.commands.hatch.AutoPlaceHatchSequenceNoDTReq;

public class VisionPlaceSequence extends CommandGroup {

    private static final Logger logger = Logger.getLogger(VisionPlaceSequence.class);

    public static double MAX_DISTANCE = 0.4;
    public static double MAX_ANGLE_ERROR_DEGREES = 3;

    public VisionPlaceSequence() {
        SmartDashboard.putNumber("VisionPlaceSequence/MAX_DISTANCE", MAX_DISTANCE);
        SmartDashboard.putNumber("VisionPlaceSequence/MAX_ANGLE_ERROR_DEGREES", MAX_ANGLE_ERROR_DEGREES);

        addSequential(new WaitUntilCommand(() ->
            isDistanceReached()
                && isAngleReached()));
        addSequential(new AutoPlaceHatchSequenceNoDTReq());
    }

    private boolean isDistanceReached() {
        return Robot.drivetrain.getDriveCommand().getLineupLocalization().getDistanceToVisionTarget() < MAX_DISTANCE;
    }

    private boolean isAngleReached() {
        return Math.abs(Robot.drivetrain.getDriveCommand().getLineupLocalization().returnAngleError(Math.PI)) < Math.toRadians(MAX_ANGLE_ERROR_DEGREES);
    }

    @Override
    protected void initialize() {
        MAX_DISTANCE = SmartDashboard.getNumber("VisionPlaceSequence/MAX_DISTANCE", MAX_DISTANCE);
        MAX_ANGLE_ERROR_DEGREES = SmartDashboard.getNumber("VisionPlaceSequence/MAX_ANGLE_ERROR_DEGREES", MAX_ANGLE_ERROR_DEGREES);
        logger.debug("Init");
    }

    @Override
    protected void execute() {

        // Debug TODO: Remove this
        boolean distanceReached = isDistanceReached();
        boolean angleReached = isAngleReached();

        String reason;
        if (!distanceReached && !angleReached) {
            reason = "Both";
        } else if (!distanceReached) {
            reason = "Distance";
        } else if (!angleReached) {
            reason = "Angle";
        } else {
            reason = "None";
        }

        logger.debug("Waiting reason: " + reason);
    }

    @Override
    protected void end() {
        logger.debug("End");
    }

    @Override
    protected void interrupted() {
        end();
    }
}
