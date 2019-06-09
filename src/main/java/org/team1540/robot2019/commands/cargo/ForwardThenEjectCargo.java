package org.team1540.robot2019.commands.cargo;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.team1540.robot2019.commands.drivetrain.simple.TankDriveForTimePercent;
import org.team1540.rooster.util.SimpleCommand;

public class ForwardThenEjectCargo extends CommandGroup {

    public ForwardThenEjectCargo() {
        addSequential(new SimpleCommand("Drive", () -> new TankDriveForTimePercent(0.2, 0.3).start()));
        addSequential(new EjectCargo());
    }
}
