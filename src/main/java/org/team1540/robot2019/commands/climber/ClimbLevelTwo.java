package org.team1540.robot2019.commands.climber;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.commands.elevator.MoveElevatorToPosition;

public class ClimbLevelTwo extends CommandGroup {

    public ClimbLevelTwo() {
        addSequential(new MoveElevatorToPosition(Tuning.elevatorClimbPosition));
        addSequential(new MoveArmsToPosition(55000));
    }
}
