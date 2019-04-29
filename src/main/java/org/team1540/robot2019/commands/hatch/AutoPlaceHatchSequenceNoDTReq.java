package org.team1540.robot2019.commands.hatch;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import org.team1540.robot2019.commands.drivetrain.TankDriveForTimePercent;
import org.team1540.robot2019.commands.elevator.MoveElevatorToZero;
import org.team1540.robot2019.commands.hatch.simple.ExtendHatchMech;
import org.team1540.robot2019.commands.hatch.simple.GrabHatch;
import org.team1540.robot2019.commands.hatch.simple.ReleaseHatch;
import org.team1540.robot2019.commands.hatch.simple.RetractHatchMech;
import org.team1540.rooster.util.SimpleCommand;

public class AutoPlaceHatchSequenceNoDTReq extends CommandGroup {

    public AutoPlaceHatchSequenceNoDTReq() {
        addSequential(new GrabHatch());
        addSequential(new ExtendHatchMech());
        addSequential(new SimpleCommand("", () -> new TankDriveForTimePercent(0.3, 0.2)));
        addSequential(new WaitCommand(0.3));
        addSequential(new WaitCommand(0.2));
        addSequential(new ReleaseHatch());
        addSequential(new WaitCommand(0.1));
        addSequential(new RetractHatchMech());
        addSequential(new WaitCommand(0.1));
        addSequential(new MoveElevatorToZero());
    }

    @Override
    protected void interrupted() {
        new ReleaseHatch().start();
        new RetractHatchMech().start();
    }
}
