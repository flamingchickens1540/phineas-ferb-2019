package org.team1540.robot2019.commands.leds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.TimedCommand;
import org.team1540.robot2019.Hardware;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.subsystems.LEDs.LEDColor;

public class BlinkLEDsAndTurnOffLimelight extends TimedCommand {

    private LEDColor color1;
    private LEDColor color2;
    private double blinkTime;

    private Timer timer = new Timer();

    boolean usingColor2 = false;

    public BlinkLEDsAndTurnOffLimelight(LEDColor color1, LEDColor color2, double blinkTime) {
        super(Tuning.ledTime);
        requires(Robot.leds);
        this.color1 = color1;
        this.color2 = color2;
        this.blinkTime = blinkTime;
    }

    @Override
    protected void initialize() {
        usingColor2 = true;
        timer.start();
        Hardware.limelight.setLeds(false);
    }

    @Override
    protected void execute() {
        if (timer.hasPeriodPassed(blinkTime)) {
            usingColor2 = !usingColor2;
        }

        Robot.leds.setColor(usingColor2 ? color2 : color1);
    }
}
