package org.team1540.robot2019.commands.cargo_intake;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.commands.elevator.MoveElevatorToZero;
import org.team1540.robot2019.commands.wrist.WristDown;
import org.team1540.rooster.util.SimpleCommand;

public class LowerThenCargoIntake extends CommandGroup {

    public LowerThenCargoIntake() {
        addParallel(new SimpleCommand("alaska's", Robot.hatch::retract, Robot.hatch));
        addSequential(new MoveElevatorToZero());
        addParallel(new AutoIntakeCargo());
        addSequential(new WristDown());
    }
}
