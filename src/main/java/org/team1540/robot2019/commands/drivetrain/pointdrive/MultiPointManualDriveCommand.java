package org.team1540.robot2019.commands.drivetrain.pointdrive;

import org.apache.log4j.Logger;
import org.team1540.robot2019.OI;
import org.team1540.robot2019.Robot;

public class MultiPointManualDriveCommand extends PointManualDriveCommand { // TODO: Make this generic

    private static final Logger logger = Logger.getLogger(MultiPointManualDriveCommand.class);

    private PercentManualLineupLocalizationAngleProvider lineupLocalization = new PercentManualLineupLocalizationAngleProvider(Robot.odometry, Robot.deepSpaceVisionTargetLocalization);
    private PointDriveAngleProvider pointDriveAngleProvider = new PointDriveAngleProvider();

    private PointAngleProvider currentAngleProvider = pointDriveAngleProvider;

    private boolean tempDisableLineup = false;

    public MultiPointManualDriveCommand() {
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
        if (OI.getPointDriveMagnitude() > 0.4
//            || Math.abs(pointDriveAngleProvider.returnAngleError()) > Math.toRadians(10)
        ) { // TODO: Make these tunable
            tempDisableLineup = false;
            startPointDrive();
        } else {
            if (tempDisableLineup) {
                return 0;
            } else {
                startLineup();
                return lineupLocalization.returnAngleError(pointDriveAngleProvider.returnAngleError(0));
            }
        }
        return currentAngleProvider.returnAngleError(0);
    }

    private void startPointDrive() {
        if (currentAngleProvider != pointDriveAngleProvider) {
            currentAngleProvider = pointDriveAngleProvider;
            justLetGoReset();
            initializeAndUpdateConfig();
            lineupLocalization.end();
        }
    }

    private void startLineup() {
        if (currentAngleProvider != lineupLocalization) {
            currentAngleProvider = lineupLocalization;
            initializeAndUpdateConfig();
        }
    }

    public boolean isLineupRunning() {
        return currentAngleProvider == lineupLocalization;
    }

    private void initializeAndUpdateConfig() {
        currentAngleProvider.initialize();
        super.applyConfig(currentAngleProvider.getPointControlConfig());
    }

    public void tempDisableLineup() {
        tempDisableLineup = true;
        logger.debug("Lineup temporarily disabled!");
    }

    public void clearTempDisableLineup() {
        tempDisableLineup = false;
        logger.debug("Lineup re-enabled!");
    }

    public boolean isLineupTempDisabled() {
        return tempDisableLineup;
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    public PercentManualLineupLocalizationAngleProvider getLineupLocalization() {
        return lineupLocalization;
    }

    public PointDriveAngleProvider getPointDriveAngleProvider() {
        return pointDriveAngleProvider;
    }
}
