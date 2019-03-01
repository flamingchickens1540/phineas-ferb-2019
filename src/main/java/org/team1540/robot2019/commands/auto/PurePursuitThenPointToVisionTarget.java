package org.team1540.robot2019.commands.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.commands.drivetrain.TankDriveForTimePercent;
import org.team1540.robot2019.commands.drivetrain.TankDriveForTimeVelocity;

public class PurePursuitThenPointToVisionTarget extends CommandGroup {

    public PurePursuitThenPointToVisionTarget() {
        addSequential(new WaitCommand(0.05)); // Wait for leds to turn on
        addSequential(new PurePursuitToVisionTarget(Robot.deepSpaceVisionTargetLocalization, Robot.odometry, this::cancel));

        addSequential(new SimplePointToVisionTarget());

//        addSequential(new TankDriveForTimeVelocity(0.6, 0.5)); // TODO: Straight driving with navx
        addSequential(new TankDriveForTimePercent(0.35, 0.25)); // TODO: Straight driving with navx
    }

    @Override
    protected void initialize() {
        Robot.limelight.setLeds(true);
    }

    @Override
    protected void initialize() {
        Robot.limelight.setLeds(true);
    }

    @Override
    protected void interrupted() {
        end(); // CommandGroups don't call end by default on interrupted
    }

    @Override
    protected void end() {
        Robot.limelight.setLeds(false);
    }
}
