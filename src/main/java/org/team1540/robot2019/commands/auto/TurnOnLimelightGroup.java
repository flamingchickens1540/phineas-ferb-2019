package org.team1540.robot2019.commands.auto;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import org.team1540.robot2019.Robot;

public class TurnOnLimelightGroup extends CommandGroup {

    public TurnOnLimelightGroup(Command command) {
        addSequential(new WaitCommand(0.05)); // Wait for leds to turn on
//        addSequential(command);
//        addSequential(new PurePursuitToVisionTarget(Robot.deepSpaceVisionTargetLocalization, Robot.odometry, this::cancel));

        addSequential(new SimplePointToVisionTarget());

//        addSequential(new TankDriveForTimeVelocity(0.6, 0.5)); // TODO: Straight driving with navx
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
