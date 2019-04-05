package org.team1540.robot2019.commands.cargo;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.apache.log4j.Logger;
import org.team1540.robot2019.Hardware;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;

public class IntakeCargo extends Command {

    private static final Logger logger = Logger.getLogger(IntakeCargo.class);
    private static double MAX_CURRENT = 8;

    public IntakeCargo() {
        requires(Robot.intake);
        SmartDashboard.putNumber("Intake/MAX_CURRENT", MAX_CURRENT);
    }

    @Override
    protected void initialize() {
        MAX_CURRENT = SmartDashboard.getNumber("Intake/MAX_CURRENT", MAX_CURRENT);
        logger.debug("Intake starting, MAX_CURRENT: " + MAX_CURRENT);
        Robot.intake.startIntaking();
    }

    @Override
    protected void end() {
        logger.debug("Intake stopping");
        Robot.intake.stop();
    }

    @Override
    protected boolean isFinished() {
        if (Robot.debugMode) {
            SmartDashboard.putNumber("PDP 6", Hardware.pdp.getCurrent(6));
        }
        if (!Tuning.isComp && this.timeSinceInitialized() > 0.5 && Hardware.pdp.getCurrent(6) > MAX_CURRENT) {
            return true;
        }
        return Robot.intake.hasBall() || isTimedOut();
    }
}
