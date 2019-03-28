package org.team1540.robot2019.commands.hatch;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.TimedCommand;
import edu.wpi.first.wpilibj.command.WaitCommand;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.commands.drivetrain.TankDriveForTimePercent;
import org.team1540.robot2019.commands.elevator.MoveElevatorToPosition;
import org.team1540.robot2019.commands.elevator.MoveElevatorToZero;
import org.team1540.robot2019.commands.hatch.simple.ExtendHatchMech;
import org.team1540.robot2019.commands.hatch.simple.GrabHatch;
import org.team1540.robot2019.commands.hatch.simple.ReleaseHatch;
import org.team1540.robot2019.commands.hatch.simple.RetractHatchMech;
import org.team1540.rooster.util.SimpleCommand;

public class PlaceHatchInLoadingStation extends CommandGroup {

    public PlaceHatchInLoadingStation() {
        addSequential(new GrabHatch());
        addSequential(new SimpleCommand("", () -> Robot.drivetrain.getDriveCommand().tempDisableLineup()));
        addSequential(new MoveElevatorToPosition(10));
        addSequential(new ExtendHatchMech());
        addSequential(new SimpleCommand("Drive", () -> new TankDriveForTimePercent(0.4, 0.2).start()));
        addSequential(new TimedCommand(0.4));
        addSequential(new MoveElevatorToPosition(2));
        addSequential(new ReleaseHatch());
        addSequential(new WaitCommand(0.1));
        addSequential(new RetractHatchMech());
        addSequential(new WaitCommand(0.1));
        addSequential(new MoveElevatorToZero());
        addSequential(new SimpleCommand("Drive", () -> new TankDriveForTimePercent(0.2, 0.3).start()));
        addSequential(new TimedCommand(0.2));
    }

    @Override
    protected void interrupted() {
        new ReleaseHatch().start();
        new RetractHatchMech().start();
    }
}