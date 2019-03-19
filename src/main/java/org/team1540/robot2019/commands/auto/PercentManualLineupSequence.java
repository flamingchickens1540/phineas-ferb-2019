package org.team1540.robot2019.commands.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.team1540.robot2019.Hardware;
import org.team1540.robot2019.Robot;

public class PercentManualLineupSequence extends CommandGroup {


    public PercentManualLineupSequence() {
//        addSequential(new SimpleConditionalCommand(() -> !Robot.limelight.isLEDsOn(), new WaitCommand(0.05))); // Wait for leds to turn on
//        addSequential(new PercentManualLineup());
        addSequential(new PercentManualLineupLocalization(Robot.odometry, Robot.deepSpaceVisionTargetLocalization));
//        SmartDashboard.setDefaultBoolean("ManualVisionToggle", true);
//        addSequential(new SimpleConditionalCommand(() -> !Robot.intake.hasBall()
//            , new PercentManualLineupLocalization(Robot.odometry, Robot.deepSpaceVisionTargetLocalization),
//            new PercentManualLineup()));
    }

    @Override
    protected void initialize() {
        Hardware.limelight.prepForVision();
    }

    @Override
    protected void interrupted() {
        end(); // CommandGroups don't call end by default on interrupted
    }

    @Override
    protected void end() {
        Hardware.limelight.prepForDriverCam();
    }
}
