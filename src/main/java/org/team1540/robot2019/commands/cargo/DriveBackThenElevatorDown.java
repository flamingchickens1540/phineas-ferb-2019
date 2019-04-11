package org.team1540.robot2019.commands.cargo;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.TimedCommand;
import org.team1540.robot2019.commands.drivetrain.simple.TankDriveForTimePercent;
import org.team1540.robot2019.commands.elevator.MoveElevatorToZero;
import org.team1540.rooster.util.SimpleCommand;

public class DriveBackThenElevatorDown extends CommandGroup {

    public DriveBackThenElevatorDown() {
        addSequential(new SimpleCommand("Drive", () -> new TankDriveForTimePercent(0.2, -0.3).start()));
        addSequential(new TimedCommand(0.3));
        addSequential(new MoveElevatorToZero());
    }

}
