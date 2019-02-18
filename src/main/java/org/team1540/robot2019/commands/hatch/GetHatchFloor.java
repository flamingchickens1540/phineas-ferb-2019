package org.team1540.robot2019.commands.hatch;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.commands.elevator.MoveElevatorToZero;
import org.team1540.robot2019.commands.wrist.WristDown;
import org.team1540.rooster.util.SimpleCommand;

public class GetHatchFloor extends CommandGroup {

    public GetHatchFloor() {
        addParallel(new RetractHatch());
        addParallel(new SimpleCommand("close grabber", Robot.hatch::release, Robot.hatch));
        addSequential(new MoveElevatorToZero());
        addSequential(new WristDown());
        addSequential(new WaitCommand(Tuning.hatchFloorTime));
        addSequential(new ExtendHatch());
        addSequential(new Command() {
            @Override
            protected boolean isFinished() {
                return false;
            }
        });
    }
}
