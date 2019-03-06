package org.team1540.robot2019.commands.auto;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.team1540.robot2019.OI;
import org.team1540.robot2019.Robot;

public class WaitForVisionTarget extends Command {

    public static double TIMEOUT = 1;

    public WaitForVisionTarget() {
        super(TIMEOUT);
    }

    @Override
    protected void initialize() {
        if (SmartDashboard.getBoolean("TurnOffLimelightWhenNotInUse", true)) {
            Robot.limelight.prepForVision();
        }
    }

    @Override
    protected boolean isFinished() {
        if (this.isTimedOut()) {
            return false;
        }
        return Robot.limelight.isTargetFound();
    }

    @Override
    protected void end() {
        if (OI.alignCommand != null && !this.isTimedOut()) {
            new RumbleForTime(OI.driver, 0.2).start();
            OI.alignCommand.start();
        } else if (SmartDashboard.getBoolean("TurnOffLimelightWhenNotInUse", true)) {
            Robot.limelight.prepForDriverCam();
        }
    }
}
