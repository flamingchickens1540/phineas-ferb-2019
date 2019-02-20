package org.team1540.robot2019.commands.drivetrain;

import edu.wpi.first.wpilibj.command.Command;
import org.team1540.robot2019.OI;
import org.team1540.robot2019.Robot;

public class PercentTankDrive extends Command {

    public PercentTankDrive() {
        requires(Robot.drivetrain);
    }

    @Override
    protected void execute() {
        double triggerValue = OI.getTankdriveForwardsAxis() - OI.getTankdriveBackwardsAxis();
        Robot.drivetrain.setLeftPercent(triggerValue - OI.getTankdriveLeftAxis());
        Robot.drivetrain.setRightPercent(triggerValue - OI.getTankdriveRightAxis());
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
