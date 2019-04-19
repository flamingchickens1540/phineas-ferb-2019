package org.team1540.robot2019.commands.climber;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import org.team1540.robot2019.Robot;
import org.team1540.rooster.util.SimpleCommand;

public class LiftGyroStabilizeLevel3Group extends CommandGroup {

    public LiftGyroStabilizeLevel3Group() {
        addSequential(new SimpleCommand("", Robot.climber::lowerCylinder));
        addSequential(new WaitCommand(2.2));
        addParallel(new LiftGyroStabilizeLevel3());
    }
}
