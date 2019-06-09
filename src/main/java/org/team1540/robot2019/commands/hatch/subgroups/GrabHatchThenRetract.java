package org.team1540.robot2019.commands.hatch.subgroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import org.team1540.robot2019.commands.elevator.MoveElevatorToZero;
import org.team1540.robot2019.commands.hatch.simple.GrabHatch;
import org.team1540.robot2019.commands.hatch.simple.RetractHatchMech;

public class GrabHatchThenRetract extends CommandGroup {

    public GrabHatchThenRetract(double hatchGrabWaitTime) {
        addSequential(new GrabHatch());
        addSequential(new WaitCommand(hatchGrabWaitTime));
        addSequential(new RetractHatchMech());
        addSequential(new MoveElevatorToZero());
    }
}
