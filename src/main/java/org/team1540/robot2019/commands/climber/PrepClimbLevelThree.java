package org.team1540.robot2019.commands.climber;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.commands.elevator.MoveElevatorToPosition;

public class PrepClimbLevelThree extends CommandGroup {

    public static boolean hasPrepLvl3 = false;

    public PrepClimbLevelThree() {
        addSequential(new MoveElevatorToPosition(Tuning.elevatorClimbPosition));
        addSequential(new MoveArmsToPosition(Tuning.climberStartPosLevel3));
        addSequential(new MoveElevatorToPosition(Tuning.elevatorDuringClimbPosition));
//        addSequential(new LiftGyroStabilizeLevel3());
    }

    @Override
    protected void end() {
        hasPrepLvl3 = true;
        PrepClimbLevelTwo.hasPrepLvl2 = false;
    }

    @Override
    protected void interrupted() {
        end();
    }

}
