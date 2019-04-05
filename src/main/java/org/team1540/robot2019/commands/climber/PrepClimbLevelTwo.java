package org.team1540.robot2019.commands.climber;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.commands.elevator.MoveElevatorToPosition;
import org.team1540.robot2019.commands.elevator.MoveElevatorToZero;

public class PrepClimbLevelTwo extends CommandGroup {

    private static double ARM_POS = Tuning.climberStartPosLevel2;
    public static boolean hasPrepLvl2 = false;
    private MoveArmsToPosition command;

    public PrepClimbLevelTwo() {
        SmartDashboard.putNumber("Level2ArmPos", ARM_POS);

        addSequential(new MoveElevatorToPosition(Tuning.elevatorClimbPosition));
        command = new MoveArmsToPosition(Tuning.climberStartPosLevel2);
        addSequential(command);
        addSequential(new MoveElevatorToZero());
//        addSequential(new LiftGyroStabilizeLevel2());
    }

    @Override
    protected void initialize() {
        ARM_POS = SmartDashboard.getNumber("Level2ArmPos", ARM_POS);
        command.setPosition(ARM_POS);
    }

    @Override
    protected void end() {
        hasPrepLvl2 = true;
    }

    @Override
    protected void interrupted() {
        end();
    }
}
