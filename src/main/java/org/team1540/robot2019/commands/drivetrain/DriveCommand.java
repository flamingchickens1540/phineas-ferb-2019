package org.team1540.robot2019.commands.drivetrain;

import org.team1540.robot2019.OI;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.commands.auto.PercentManualLineupLocalizationAngleProvider;
import org.team1540.robot2019.commands.auto.PointAngleProvider;
import org.team1540.robot2019.commands.auto.PointControlConfig;
import org.team1540.robot2019.commands.auto.PointManualDriveCommand;

public class DriveCommand extends PointManualDriveCommand { // TODO: Make this generic

    private PercentManualLineupLocalizationAngleProvider lineupLocalization = new PercentManualLineupLocalizationAngleProvider(Robot.odometry, Robot.deepSpaceVisionTargetLocalization);
    private PointDriveAngleProvider pointDriveAngleProvider = new PointDriveAngleProvider();

    private PointAngleProvider currentAngleProvider = pointDriveAngleProvider;

    private boolean tempDisableLineup = false;

    public DriveCommand() {
        super();
    }

    public void pointNextReset() {
        lineupLocalization.pointNextReset();
    }

    public void justLetGoReset() {
        lineupLocalization.justLetGoReset();
    }

    @Override
    protected PointControlConfig initializeAndGetConfig() {
        currentAngleProvider.initialize();
        return currentAngleProvider.getPointControlConfig();
    }

    @Override
    protected double returnAngleError() {
        if (OI.getPointDriveMagnitude() > 0.4) {
            tempDisableLineup = false;
            if (currentAngleProvider != pointDriveAngleProvider) {
                currentAngleProvider = pointDriveAngleProvider;
                justLetGoReset();
                initializeAndUpdateConfig();
            }
        } else if (!tempDisableLineup && currentAngleProvider != lineupLocalization) {
            currentAngleProvider = lineupLocalization;
            initializeAndUpdateConfig();
        }
        return currentAngleProvider.returnAngleError();
    }

    private void initializeAndUpdateConfig() {
        currentAngleProvider.initialize();
        super.applyConfig(currentAngleProvider.getPointControlConfig());
    }

    public void tempDisableLineup() {
        tempDisableLineup = true;
        if (currentAngleProvider != pointDriveAngleProvider) {
            currentAngleProvider = pointDriveAngleProvider;
            initializeAndUpdateConfig();
        }
        logger.debug("Lineup temporarily disabled!");
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
