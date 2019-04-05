package org.team1540.robot2019.commands.climber;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.commands.elevator.MoveElevatorToPosition;
import org.team1540.robot2019.commands.elevator.MoveElevatorToZero;

public class PrepClimbLevelTwo extends CommandGroup {

    public static boolean hasPrepLvl2 = false;

    public PrepClimbLevelTwo() {
        addSequential(new MoveElevatorToPosition(Tuning.elevatorClimbPosition));
        addSequential(new MoveArmsToPosition(Tuning.climberStartPosLevel2));
        addSequential(new MoveElevatorToZero());
//        addSequential(new LiftGyroStabilizeLevel2());
    }

    @Override
    protected void end() {
        hasPrepLvl2 = true;
    }

    @Override
    protected void interrupted() {
        end();
    }
}
