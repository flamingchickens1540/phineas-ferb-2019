package org.team1540.robot2019.commands.hatch;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.TimedCommand;
import edu.wpi.first.wpilibj.command.WaitCommand;
import org.team1540.robot2019.commands.drivetrain.TankDriveForTimePercent;
import org.team1540.robot2019.commands.elevator.MoveElevatorToZero;
import org.team1540.rooster.util.SimpleCommand;

public class TestPlaceHatch extends CommandGroup {

    public TestPlaceHatch() {
        addSequential(new GrabHatch());
        addSequential(new ExtendHatchMech());
        addSequential(new SimpleCommand("Drive", () -> new TankDriveForTimePercent(0.3, 0.3).start()));
        addSequential(new TimedCommand(0.3));
        addSequential(new WaitCommand(0.2));
        addSequential(new ReleaseHatch());
        addSequential(new WaitCommand(0.1));
        addSequential(new RetractHatchMech());
        addSequential(new WaitCommand(0.1));
        addSequential(new MoveElevatorToZero());
    }

    @Override
    protected void interrupted() {
        new ReleaseHatch().start();
        new RetractHatchMech().start();
    }
}
