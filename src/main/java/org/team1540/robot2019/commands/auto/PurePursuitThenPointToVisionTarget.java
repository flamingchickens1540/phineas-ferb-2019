package org.team1540.robot2019.commands.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.commands.drivetrain.DriveForTimeVelocity;
import org.team1540.rooster.util.SimpleCommand;

public class PurePursuitThenPointToVisionTarget extends CommandGroup {

    public PurePursuitThenPointToVisionTarget() {
        addSequential(new SimpleCommand("LEDs ON", () -> Robot.limelight.setLeds(true)));
        addSequential(new WaitCommand(0.05)); // Wait for leds to turn on
        addSequential(new PurePursuitToVisionTarget(Robot.deepSpaceVisionTargetLocalization, Robot.odometry, this::cancel));

        addSequential(new SimplePointToVisionTarget());

        addSequential(new DriveForTimeVelocity(0.6, 0.5)); // TODO: Straight driving with navx
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
