package org.team1540.robot2019.commands.leds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.TimedCommand;
import org.team1540.robot2019.Hardware;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;

public class BlinkLEDsAndTurnOffLimelight extends TimedCommand {

    private double blinkTime;

    private Timer timer = new Timer();

    boolean usingColor2 = false;

    public BlinkLEDsAndTurnOffLimelight() {
        super(Tuning.ledTime);
        requires(Robot.leds);
    }

    @Override
    protected void initialize() {
        usingColor2 = true;
        timer.start();
        Hardware.limelight.setLeds(false);
    }

    @Override
    protected void execute() {
        if (timer.hasPeriodPassed(Tuning.ledStrobeTime)) {
            usingColor2 = !usingColor2;
        }

        Robot.leds.set(usingColor2 ? Tuning.limelightOffLEDs2 : Tuning.limelightOffLEDs1);
    }
}
