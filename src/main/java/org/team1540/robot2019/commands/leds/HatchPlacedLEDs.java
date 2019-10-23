package org.team1540.robot2019.commands.leds;

import edu.wpi.first.wpilibj.command.TimedCommand;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;

public class HatchPlacedLEDs extends TimedCommand {

    public HatchPlacedLEDs() {
        super(0.5);
        requires(Robot.leds);
    }

    @Override
    protected void initialize() {
        Robot.leds.set(Tuning.hatchPlacedLEDs);
    }
}
