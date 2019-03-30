package org.team1540.robot2019.commands.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.team1540.robot2019.commands.drivetrain.AutoLineupAndDrive;
import org.team1540.robot2019.commands.drivetrain.TankDriveForTimePercent;
import org.team1540.robot2019.commands.hatch.SensorGrabHatchSequence;

public class AutoGrabSequence extends CommandGroup {

    public AutoGrabSequence() {
        AutoLineupAndDrive autoLineupAndDrive = new AutoLineupAndDrive();
        addParallel(new SensorGrabHatchSequence(autoLineupAndDrive::enableEndFlag));
        addSequential(autoLineupAndDrive);
        addSequential(new TankDriveForTimePercent(0.5, -0.02));
        addSequential(new TankDriveForTimePercent(1.5, -0.2));
    }
}
