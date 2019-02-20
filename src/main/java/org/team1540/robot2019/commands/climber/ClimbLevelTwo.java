package org.team1540.robot2019.commands.climber;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.commands.elevator.MoveElevatorToPosition;
import org.team1540.robot2019.commands.elevator.MoveElevatorToZero;

public class ClimbLevelTwo extends CommandGroup {

    public ClimbLevelTwo() {
        addSequential(new MoveElevatorToPosition(Tuning.elevatorClimbPosition));
        addSequential(new MoveArmsToPosition(Tuning.climberStartPosLevel2));
        addSequential(new MoveElevatorToZero());
        addSequential(new ExtendGyroStabilizeLevel2());
    }
}
