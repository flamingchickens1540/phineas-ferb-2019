package org.team1540.robot2019.commands.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.team1540.robot2019.commands.drivetrain.simple.TankDriveForTimePercent;
import org.team1540.robot2019.commands.drivetrain.simple.TankDriveForTimeVelocity;
import org.team1540.robot2019.commands.hatch.sensor.SensorGrabHatchSequence;

public class DriveSensorGrabHatchSequence extends CommandGroup {

    public DriveSensorGrabHatchSequence() {
        AutoLineupAndDrive autoLineupAndDrive = new AutoLineupAndDrive();
        addParallel(new SensorGrabHatchSequence(autoLineupAndDrive::enableEndFlag));
        addSequential(autoLineupAndDrive);
        addSequential(new TankDriveForTimeVelocity(0.4, 0));
        addSequential(new TankDriveForTimePercent(0.25, -0.3, true));
    }
}
