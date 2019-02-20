package org.team1540.robot2019.commands.climber;

import edu.wpi.first.wpilibj.command.PIDCommand;
import org.team1540.robot2019.Hardware;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;

public class ExtendGyroStabilize extends PIDCommand {

    public ExtendGyroStabilize() {
        super(Tuning.climberGyroP, Tuning.climberGyroI, Tuning.climberGyroD);
        requires(Robot.climber);
    }

    @Override
    protected void initialize() {
        Robot.climber.cylinderDown();
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected double returnPIDInput() {
        return Hardware.navx.getRawPitchDegrees();
    }

    @Override
    protected void usePIDOutput(double v) {
        Robot.climber.setArms(v + Tuning.climberGyroFF);
    }
}
