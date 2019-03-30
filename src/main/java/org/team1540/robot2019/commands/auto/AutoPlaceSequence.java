package org.team1540.robot2019.commands.auto;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import org.team1540.robot2019.commands.drivetrain.AutoLineupAndDrive;
import org.team1540.robot2019.commands.drivetrain.TankDriveForTimePercent;
import org.team1540.robot2019.commands.hatch.AutoPlaceHatchSequence;

public class AutoPlaceSequence extends CommandGroup {

    public AutoPlaceSequence() {
        AutoLineupAndDrive autoLineupAndDrive = new AutoLineupAndDrive();
        addParallel(new Command() {
            @Override
            protected boolean isFinished() {
                if (autoLineupAndDrive.getDistanceToVisionTarget() < 0.5) {
                    autoLineupAndDrive.enableEndFlag();
                    return true;
                }
                return false;
            }
        });
        addSequential(autoLineupAndDrive);
        addSequential(new AutoPlaceHatchSequence());
        addSequential(new TankDriveForTimePercent(1.5, -0.2));
    }
}
