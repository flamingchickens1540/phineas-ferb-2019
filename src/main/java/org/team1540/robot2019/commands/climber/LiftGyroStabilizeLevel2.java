package org.team1540.robot2019.commands.climber;

import edu.wpi.first.wpilibj.command.Command;
import org.team1540.robot2019.Hardware;
import org.team1540.robot2019.OI;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;

public class LiftGyroStabilizeLevel2 extends Command {

    public LiftGyroStabilizeLevel2() {
        requires(Robot.climber);
    }

    @Override
    protected void initialize() {
    }

    @Override
    protected void execute() {
        if (Hardware.navx.getRawPitchDegrees() > 0) {
            Robot.climber.cylinderDown();
        }
        else {
            Robot.climber.cylinderUp();
        }

        if (Robot.climber.getPosition() < Tuning.climberArmsTooFar) {
            Robot.climber.setArms(OI.getClimbAxis());
        }
        else {
            Robot.climber.setArms(0);
        }
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

}
