package org.team1540.robot2019.commands.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.commands.drivetrain.TankDriveForTimeVelocity;

public class SimpleTwoStageLineupSequence extends CommandGroup {

    public SimpleTwoStageLineupSequence() {
        addSequential(new WaitCommand(0.05)); // Wait for leds to turn on
        addSequential(new PurePursuitToVisionTarget(Robot.deepSpaceVisionTargetLocalization, Robot.odometry, this::cancel)); // TODO: Replace this with getGroup().cancel()
        addSequential(new SimplePointToVisionTarget());
        addSequential(new TankDriveForTimeVelocity(0.6, 0.5)); // TODO: Straight driving with navx
    }

    @Override
    protected void initialize() {
        if (SmartDashboard.getBoolean("TurnOffLimelightWhenNotInUse", true)) {
            Robot.limelight.prepForVision();
        }
    }

    @Override
    protected void interrupted() {
        end(); // CommandGroups don't call end by default on interrupted
    }

    @Override
    protected void end() {
        if (SmartDashboard.getBoolean("TurnOffLimelightWhenNotInUse", true)) {
            Robot.limelight.prepForDriverCam();
        }
    }
}
