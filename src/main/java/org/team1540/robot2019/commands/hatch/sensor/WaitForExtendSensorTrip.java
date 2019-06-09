package org.team1540.robot2019.commands.hatch.sensor;

import edu.wpi.first.wpilibj.command.Command;
import org.team1540.robot2019.Robot;

public class WaitForExtendSensorTrip extends Command {

    @Override
    protected void initialize() {
        Robot.hatch.clearExtendSensor();
    }

    @Override
    protected boolean isFinished() {
        if (Robot.hatch.wasExtendSensorTripped()) {
            Robot.hatch.clearExtendSensor();
            return true;
        }
        return false;
    }
}
