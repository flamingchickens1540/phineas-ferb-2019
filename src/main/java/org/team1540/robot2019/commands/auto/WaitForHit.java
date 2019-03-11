package org.team1540.robot2019.commands.auto;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Command;
import org.apache.log4j.Logger;
import org.team1540.robot2019.Hardware;

public class WaitForHit extends Command {

    public static final Logger logger = Logger.getLogger(WaitForHit.class);

    private static double HIT_JERK_THRESHOLD = 0.4;

    private double lastAccelX = 0;

    @Override
    protected void initialize() {
        NetworkTableInstance.getDefault().getTable("HitDetection").getEntry("HIT_JERK_THRESHOLD").setDefaultNumber(HIT_JERK_THRESHOLD);
        HIT_JERK_THRESHOLD = NetworkTableInstance.getDefault().getTable("HitDetection").getEntry("HIT_JERK_THRESHOLD").getDouble(HIT_JERK_THRESHOLD);
    }

    @Override
    protected boolean isFinished() {
        double currentAccelX = Hardware.navx.getAccelX();
        double currentJerkX = currentAccelX - lastAccelX; // TODO: This needs to account for delta time (can't rely on 20ms loop times)
        lastAccelX = currentAccelX;
        NetworkTableInstance.getDefault().getTable("HitDetection").getEntry("CurrentJerk").setNumber(currentJerkX);
        boolean didHit = Math.abs(currentJerkX) > HIT_JERK_THRESHOLD;
        if (didHit) {
            logger.debug("Hit detected, command ending!");
        }
        return didHit;
    }

    @Override
    protected void interrupted() {
        logger.debug("Hit detect command was interrupted!");
    }
}
