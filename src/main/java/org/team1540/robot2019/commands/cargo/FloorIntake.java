package org.team1540.robot2019.commands.cargo;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.commands.elevator.MoveElevatorToZero;
import org.team1540.robot2019.commands.hatch.RetractHatch;
import org.team1540.robot2019.commands.wrist.WristDown;
import org.team1540.rooster.util.SimpleCommand;

public class FloorIntake extends CommandGroup {

    public FloorIntake() {
        addParallel(new RetractHatch());
        addSequential(new MoveElevatorToZero());
        addParallel(new Intake());
        addSequential(new WristDown());
    }
}
