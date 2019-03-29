package org.team1540.robot2019.commands.drivetrain;

import org.apache.log4j.Logger;
import org.team1540.robot2019.OI;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.commands.auto.PercentManualLineupLocalizationAngleProvider;
import org.team1540.robot2019.commands.auto.PointAngleProvider;
import org.team1540.robot2019.commands.auto.PointControlConfig;
import org.team1540.robot2019.commands.auto.PointManualDriveCommand;

public class DriveCommand extends PointManualDriveCommand { // TODO: Make this generic

    private static final Logger logger = Logger.getLogger(DriveCommand.class);

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
            startPointDrive();
        } else {
            if (!tempDisableLineup) {
                startLineup();
            } else {
                return 0;
            }
        }
        return currentAngleProvider.returnAngleError();
    }

    private void startPointDrive() {
        if (currentAngleProvider != pointDriveAngleProvider) {
            currentAngleProvider = pointDriveAngleProvider;
            justLetGoReset();
            initializeAndUpdateConfig();
        }
    }

    private void startLineup() {
        if (currentAngleProvider != lineupLocalization) {
            currentAngleProvider = lineupLocalization;
            initializeAndUpdateConfig();
        }
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

    @Override
    protected boolean isFinished() {
        return false;
    }
}
