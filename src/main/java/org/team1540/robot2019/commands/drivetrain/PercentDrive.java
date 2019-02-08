package org.team1540.robot2019.commands.drivetrain;

import edu.wpi.first.wpilibj.command.Command;
import org.team1540.robot2019.OI;
import org.team1540.robot2019.subsystems.Drivetrain;

public class PercentDrive extends Command {

    private final Drivetrain dt;

    public PercentDrive(Drivetrain dt) {
        this.dt = dt;
        requires(dt);
//    dt.reset();
    }

    @Override
    protected void execute() {
        double triggerValue = OI.getTankdriveForwardsAxis() - OI.getTankdriveBackwardsAxis();
        dt.setLeftPercent(triggerValue - OI.getTankdriveLeftAxis());
        dt.setRightPercent(triggerValue - OI.getTankdriveRightAxis());
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
