package org.team1540.robot2019.commands.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.team1540.robot2019.Robot;

public class PurePursuitThenPointToVisionTarget extends CommandGroup {

    public PurePursuitThenPointToVisionTarget() {
        addSequential(new WaitCommand(0.05)); // Wait for leds to turn on
//        addSequential(new PurePursuitToVisionTarget(Robot.deepSpaceVisionTargetLocalization, Robot.odometry, this::cancel));

        addSequential(new SimplePointToVisionTargetAndManualDrive());
//        addSequential(new SimplePointToVisionTarget());

//        addSequential(new TankDriveForTimeVelocity(0.6, 0.5)); // TODO: Straight driving with navx
    }

    @Override
    protected void initialize() {
        if (SmartDashboard.getBoolean("TurnOffLimelightWhenNotInUse", true)) {
            Robot.limelight.setLeds(true);
        }
    }

    @Override
    protected void interrupted() {
        end(); // CommandGroups don't call end by default on interrupted
    }

    @Override
    protected void end() {
        if (SmartDashboard.getBoolean("TurnOffLimelightWhenNotInUse", true)) {
            Robot.limelight.setLeds(false);
        }    }
}
