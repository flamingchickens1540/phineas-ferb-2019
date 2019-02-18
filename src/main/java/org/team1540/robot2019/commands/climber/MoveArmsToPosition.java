package org.team1540.robot2019.commands.climber;

import edu.wpi.first.wpilibj.command.Command;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;

public class MoveArmsToPosition extends Command {

    private double setpoint;

    public MoveArmsToPosition(double pos) {
        requires(Robot.climber);
        setpoint = pos;
    }

    @Override
    protected void initialize() {
        Robot.climber.setArmPosition(setpoint);
    }

    @Override
    protected void end() {
        Robot.climber.setArms(0);
    }

    @Override
    protected boolean isFinished() {
        return Math.abs(Robot.climber.getPosition() - setpoint)
            < Tuning.climberTolerance;
    }
}
