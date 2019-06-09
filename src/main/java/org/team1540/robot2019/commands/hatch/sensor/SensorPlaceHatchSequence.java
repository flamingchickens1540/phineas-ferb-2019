package org.team1540.robot2019.commands.hatch.sensor;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import org.team1540.robot2019.commands.elevator.MoveElevatorToZero;
import org.team1540.robot2019.commands.hatch.simple.ExtendHatchMech;
import org.team1540.robot2019.commands.hatch.simple.GrabHatch;
import org.team1540.robot2019.commands.hatch.simple.ReleaseHatch;
import org.team1540.robot2019.commands.hatch.simple.RetractHatchMech;

public class SensorPlaceHatchSequence extends CommandGroup {

    public SensorPlaceHatchSequence() {
        addSequential(new GrabHatch());
        addSequential(new ExtendHatchMech());
        addSequential(new WaitCommand(0.5));
        addSequential(new WaitForExtendSensorTrip());
        addSequential(new ReleaseHatch());
        addSequential(new WaitCommand(0.2));
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
