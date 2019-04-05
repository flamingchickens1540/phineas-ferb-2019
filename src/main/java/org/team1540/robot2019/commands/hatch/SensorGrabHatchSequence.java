package org.team1540.robot2019.commands.hatch;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.commands.elevator.MoveElevatorToPosition;
import org.team1540.robot2019.commands.elevator.MoveElevatorToZero;
import org.team1540.robot2019.commands.hatch.simple.ExtendHatchMech;
import org.team1540.robot2019.commands.hatch.simple.GrabHatch;
import org.team1540.robot2019.commands.hatch.simple.ReleaseHatch;
import org.team1540.robot2019.commands.hatch.simple.RetractHatchMech;
import org.team1540.robot2019.commands.wrist.WristUp;
import org.team1540.rooster.util.SimpleCommand;

public class SensorGrabHatchSequence extends CommandGroup {

    public SensorGrabHatchSequence() {
        this(null);
    }

    public SensorGrabHatchSequence(Runnable onSensorTrip) {
        addSequential(new ReleaseHatch());
        addSequential(new RetractHatchMech());
        addSequential(new WristUp());
        addParallel(new MoveElevatorToPosition(Tuning.elevatorHatchIntakePosition));
        addSequential(new ExtendHatchMech());
        addSequential(new WaitCommand(0.5));
        addSequential(new WaitForExtendSensorTrip());
        addSequential(new SimpleCommand("", () -> {
            if (onSensorTrip != null) {
                onSensorTrip.run();
            }
        }));
        addSequential(new GrabHatch());
        addSequential(new WaitCommand(0.5));
        addSequential(new RetractHatchMech());
        addSequential(new WaitCommand(0.1));
        addSequential(new MoveElevatorToZero());
    }
//
//    @Override
//    protected void interrupted() {
//        new RetractHatchMech().start();
//    }
}
