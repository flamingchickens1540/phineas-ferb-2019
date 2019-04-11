package org.team1540.robot2019.commands.hatch;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutoGrabHatchSequence extends CommandGroup {

    public AutoGrabHatchSequence() {
        this(null);
    }

    public AutoGrabHatchSequence(Runnable onSensorTrip) {
        // this is outdated
//        addParallel(new MoveElevatorToPosition(Tuning.elevatorHatchIntakePosition));
//        addSequential(new ReleaseHatch());
//        addSequential(new ExtendHatchMech());
//        addSequential(new WaitCommand(0.2));
//        addSequential(new GrabHatch());
//        addSequential(new WaitCommand(0.5));
//        addSequential(new RetractHatchMech());
//        addSequential(new WaitCommand(0.1));
//        addSequential(new MoveElevatorToZero());
    }
}
