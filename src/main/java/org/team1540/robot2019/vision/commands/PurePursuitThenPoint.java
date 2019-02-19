package org.team1540.robot2019.vision.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.commands.drivetrain.DriveForTimeVelocity;
import org.team1540.rooster.util.SimpleCommand;

public class PurePursuitThenPoint extends CommandGroup {

    public PurePursuitThenPoint() {
        addSequential(new SimpleCommand("LEDs ON", () -> Robot.limelight.setLeds(true)));
        addSequential(new WaitCommand(0.05)); // Wait for leds to turn on
        addSequential(new PurePursuitLineup(Robot.limelightLocalization, Robot.wheelOdometry, this::cancel));

        addSequential(new PointLineupSimple());

        addSequential(new DriveForTimeVelocity(0.6, 0.5)); // TODO: Straight driving with navx
    }

    @Override
    protected void interrupted() {
        end();
    }

    @Override
    protected void end() {
        Robot.limelight.setLeds(false);
    }
}
