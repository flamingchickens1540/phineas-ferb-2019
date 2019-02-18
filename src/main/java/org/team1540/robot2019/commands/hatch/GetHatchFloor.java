package org.team1540.robot2019.commands.hatch;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.commands.elevator.MoveElevatorToZero;
import org.team1540.robot2019.commands.wrist.WristDown;
import org.team1540.rooster.util.SimpleCommand;

public class GetHatchFloor extends CommandGroup {

    public GetHatchFloor() {
        addSequential(new SimpleCommand("in", Robot.hatch::retract, Robot.hatch));
        addSequential(new SimpleCommand("close", Robot.hatch::release, Robot.hatch));
        addSequential(new MoveElevatorToZero());
        addSequential(new WristDown());
        // TODO: Shouldn't this not extend? Just let grab hatch do this:
        addSequential(new SimpleCommand("out", Robot.hatch::extend, Robot.hatch));
        addSequential(new Command() {
            @Override
            protected boolean isFinished() {
                return false;
            }
        });
    }
}
