package org.team1540.robot2019.commands.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.team1540.robot2019.commands.drivetrain.AutoLineupAndDrive;
import org.team1540.robot2019.commands.drivetrain.TankDriveForTimePercent;
import org.team1540.robot2019.commands.hatch.AutoPlaceHatchSequence;

public class AutoPlaceSequence extends CommandGroup {

    public AutoPlaceSequence() {
        AutoLineupAndDrive autoLineupAndDrive = new AutoLineupAndDrive();
        addParallel(new WaitUntilCommand(() -> autoLineupAndDrive.getDistanceToVisionTarget() < 0.5, autoLineupAndDrive::enableEndFlag));
        addSequential(autoLineupAndDrive);
        addSequential(new AutoPlaceHatchSequence());
        addSequential(new TankDriveForTimePercent(1.5, -0.2));
    }
}
