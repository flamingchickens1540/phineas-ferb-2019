package org.team1540.robot2019.commands.drivetrain;

import org.team1540.robot2019.OI;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.commands.auto.PercentManualLineupLocalizationAngleProvider;
import org.team1540.robot2019.commands.auto.PointAngleProvider;
import org.team1540.robot2019.commands.auto.PointControlConfig;
import org.team1540.robot2019.commands.auto.PointManualDriveCommand;

public class PointLineupDrive extends PointManualDriveCommand {

    private PointAngleProvider lineupLocalization = new PercentManualLineupLocalizationAngleProvider(Robot.odometry, Robot.deepSpaceVisionTargetLocalization);
    private PointAngleProvider pointDriveAngleProvider = new PointDriveAngleProvider();

    private PointAngleProvider currentAngleProvider = pointDriveAngleProvider;

    private boolean tempDisableLineup = false;

    public PointLineupDrive() {
        super();
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
                currentAngleProvider.initialize();
                super.applyConfig(currentAngleProvider.getPointControlConfig());
            }
        } else if (!tempDisableLineup && currentAngleProvider != lineupLocalization) {
            currentAngleProvider = lineupLocalization;
            currentAngleProvider.initialize();
            super.applyConfig(currentAngleProvider.getPointControlConfig());
        }
        return currentAngleProvider.returnAngleError();
    }

    public void tempDisableLineup() {
        tempDisableLineup = true;
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
