package org.team1540.robot2019.commands.elevator;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.team1540.robot2019.Robot;
import org.team1540.rooster.util.SimpleCommand;

public class MoveElevatorToZero extends CommandGroup {

    public MoveElevatorToZero() {
        addSequential(new MoveElevatorToPosition(0));
//    addSequential(new ZeroElevator());
        addSequential(new SimpleCommand("Stop Elevator", Robot.elevator::prettyPleaseStopBothTheLeftAndTheRightElevatorMotorsSoWeDontWastePowerCauseWeDontHavePowerManagement, Robot.elevator));
    }
}
