package org.team1540.robot2019.commands.hatch.testing;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import org.team1540.robot2019.commands.drivetrain.simple.Wiggle;
import org.team1540.robot2019.commands.elevator.MoveElevatorToZero;
import org.team1540.robot2019.commands.hatch.simple.GrabHatch;
import org.team1540.robot2019.commands.hatch.simple.RetractHatchMech;
import org.team1540.rooster.util.SimpleCommand;

public class WiggleAndGrabHatch extends CommandGroup { // do do do, doo, doo, doo

    public WiggleAndGrabHatch() {
        addSequential(new SimpleCommand("", () -> new Wiggle().start()));
        addSequential(new WaitCommand(Wiggle.TOTAL_TIME));
        addSequential(new WaitCommand(0.3));
        addSequential(new GrabHatch());
        addSequential(new WaitCommand(0.3));
        addSequential(new RetractHatchMech());
        addSequential(new WaitCommand(0.5));
        addSequential(new MoveElevatorToZero());
    }
}
