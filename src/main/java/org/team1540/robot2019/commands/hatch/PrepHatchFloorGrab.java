package org.team1540.robot2019.commands.hatch;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.commands.elevator.MoveElevatorToZero;
import org.team1540.robot2019.commands.wrist.WristDown;

public class PrepHatchFloorGrab extends CommandGroup {

    public PrepHatchFloorGrab() {
        addParallel(new StowHatchMech());
        addSequential(new MoveElevatorToZero());
        addSequential(new WristDown());
        addSequential(new WaitCommand(Tuning.hatchPrepFloorWaitTime));
        addSequential(new ExtendHatchMech());
        addSequential(new Command() {
            @Override
            protected boolean isFinished() {
                return false;
            }
        });
    }
}
