package org.team1540.robot2019.commands.cargo;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.team1540.robot2019.commands.elevator.MoveElevatorToZero;

public class ForwardThenEjectCargo extends CommandGroup {

    public ForwardThenEjectCargo() {
//        addSequential(new SimpleCommand("Drive", () -> new TankDriveForTimePercent(0.2, 0.3).start()));
        addSequential(new EjectCargo());
        addSequential(new MoveElevatorToZero());
    }
}
