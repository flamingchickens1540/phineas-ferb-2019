package org.team1540.robot2019.commands.leds;

import edu.wpi.first.wpilibj.command.Command;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.subsystems.LEDs.LEDColor;

public class GamePieceLEDs extends Command {

    public GamePieceLEDs() {
        requires(Robot.leds);
    }

    @Override
    protected void execute() {
        LEDColor color;
        if (Robot.intake.hasBall()) {
            color = LEDColor.RED;
        } else if (!Robot.hatch.hasNoHatch()) {
            color = LEDColor.BLUE;
        } else {
            color = LEDColor.OFF;
        }

        Robot.leds.setColor(color);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
