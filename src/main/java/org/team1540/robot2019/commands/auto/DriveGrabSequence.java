package org.team1540.robot2019.commands.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.team1540.robot2019.commands.drivetrain.AutoLineupAndDrive;
import org.team1540.robot2019.commands.drivetrain.TankDriveForTimePercent;
import org.team1540.robot2019.commands.drivetrain.TankDriveForTimeVelocity;
import org.team1540.robot2019.commands.hatch.SensorGrabHatchSequence;

public class DriveGrabSequence extends CommandGroup {

    public DriveGrabSequence() {
        AutoLineupAndDrive autoLineupAndDrive = new AutoLineupAndDrive();
        addParallel(new SensorGrabHatchSequence(autoLineupAndDrive::enableEndFlag));
        addSequential(autoLineupAndDrive);
        addSequential(new TankDriveForTimeVelocity(0.4, 0));
        addSequential(new TankDriveForTimePercent(0.25, -0.3, true));
    }
}
