package org.team1540.robot2019.vision.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.commands.drivetrain.DriveForTimeVelocity;
import org.team1540.rooster.util.SimpleCommand;

public class PurePursuitThenPoint extends CommandGroup {

    public PurePursuitThenPoint() {
        addSequential(new SimpleCommand("ON", () -> Robot.limelight.setLeds(true)));
        addSequential(new WaitCommand(0.05));
        addSequential(new PurePursuitLineup(Robot.limelightLocalization, Robot.wheelOdometry));

        addSequential(new PointLineupSimple());

        addSequential(new DriveForTimeVelocity(0.6, 0.5));
        addSequential(new SimpleCommand("OFF", () -> Robot.limelight.setLeds(false)));
    }

    @Override
    protected void interrupted() {
        Robot.limelight.setLeds(false);
    }
}
