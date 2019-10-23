package org.team1540.robot2019.commands.leds;

import edu.wpi.first.wpilibj.command.TimedCommand;
import org.team1540.robot2019.Hardware;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;

public class BlinkLEDsAndTurnOffLimelight extends TimedCommand {

    public BlinkLEDsAndTurnOffLimelight() {
        super(Tuning.ledStrobeTime);
        requires(Robot.leds);
    }

    @Override
    protected void initialize() {
        Hardware.limelight.setLeds(false);
        Robot.leds.set(Tuning.turnOffLimelightLEDs);
    }
}