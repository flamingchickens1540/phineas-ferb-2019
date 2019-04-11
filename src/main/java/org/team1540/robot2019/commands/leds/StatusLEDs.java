package org.team1540.robot2019.commands.leds;

import edu.wpi.first.wpilibj.command.Command;
import org.team1540.robot2019.Hardware;
import org.team1540.robot2019.Robot;

public class StatusLEDs extends Command {

    public StatusLEDs() {
        requires(Robot.leds);
    }

    @Override
    protected void execute() {
        if (Hardware.limelight.isTargetFound()) {
            Robot.leds.setRaw(false, false, true);
        } else if (Robot.cargoMech.hasBall()) {
            Robot.leds.setRaw(true, false, false);
        } else if (!Robot.hatch.isReleased()) {
            Robot.leds.setRaw(false, true, false);
        } else {
            Robot.leds.setRaw(false, false, false);
        }

    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
