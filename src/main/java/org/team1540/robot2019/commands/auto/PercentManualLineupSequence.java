package org.team1540.robot2019.commands.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.commands.drivetrain.PointDrive;
import org.team1540.rooster.util.SimpleConditionalCommand;

public class PercentManualLineupSequence extends CommandGroup {


    public PercentManualLineupSequence() {
//        addSequential(new WaitCommand(0.05)); // Wait for leds to turn on

//        SmartDashboard.setDefaultBoolean("ManualVisionToggle", true);
        addSequential(new SimpleConditionalCommand(() -> !Robot.intake.hasBall()
//            ^ SmartDashboard.getBoolean("ManualVisionToggle", true)
            , new PercentManualLineupLocalization(Robot.odometry, Robot.deepSpaceVisionTargetLocalization),
            new PercentManualLineup()));
    }

    @Override
    protected void initialize() {
        Robot.limelight.prepForVision();
    }

    @Override
    protected void interrupted() {
        end(); // CommandGroups don't call end by default on interrupted
    }

    @Override
    protected void end() {
        PointDrive.manualResetGoal();
        Robot.limelight.prepForDriverCam();
    }
}
