package org.team1540.robot2019.commands.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.commands.elevator.MoveElevatorToZero;
import org.team1540.robot2019.commands.hatch.PlaceHatch;

public class PlaceHatchThenDown extends CommandGroup {

    public PlaceHatchThenDown() {
        addSequential(new PlaceHatch());
        addSequential(new WaitCommand(Tuning.hatchEjectThenDownTime));
        addSequential(new MoveElevatorToZero());
    }

}
