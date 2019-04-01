package org.team1540.robot2019.wrappers;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import org.team1540.robot2019.Hardware;
import org.team1540.robot2019.Robot;
import org.team1540.rooster.util.SimpleCommand;

public class SwitchFilterButton extends CommandGroup {

    public SwitchFilterButton(int pipeline) {
        addSequential(new SimpleCommand("", () -> Hardware.limelight.setPipeline(pipeline)));
        addSequential(new WaitCommand(0.05));
        addSequential(new SimpleCommand("", Robot.drivetrain.getDriveCommand()::pointNextReset));
    }
}
