package org.team1540.robot2019.commands.climber;

import edu.wpi.first.wpilibj.command.Command;
import org.team1540.robot2019.OI;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;

public class JoystickClimberArms extends Command {

    public JoystickClimberArms() {
        requires(Robot.climber);
    }

    @Override
    protected void execute() {
        if (Robot.climber.getArmsPosition() < Tuning.climberArmsFwdLimit || OI.getManualClimberArmsAxis() < 0) {
            Robot.climber.setArms(OI.getManualClimberArmsAxis());
        } else {
            Robot.climber.setArms(0);
        }
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
