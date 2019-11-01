package org.team1540.robot2019;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.command.Command;
import java.util.ArrayList;
import java.util.List;

public class CameraFrequencyTimer extends Command {

    private final DigitalOutput digitalOutput;
    private static final List<Integer> callbackIds = new ArrayList<>();
    private static final List<Double> rateTimes = new ArrayList<>();
    private static int count;
    private static double lastDouble;

    public CameraFrequencyTimer(int ledChannel) {
        digitalOutput = new DigitalOutput(ledChannel);
    }

    @Override
    protected void initialize() {
        for (int i = 0; i < 20; i++) {
            System.out.print("Timer started!! ");
        }
        callbackIds.add(Hardware.limelight.registerTxCallback((entryNotification -> {
            double aDouble = entryNotification.value.getDouble();
            if (aDouble != 0 && lastDouble != aDouble) {
                System.out.println(this.timeSinceInitialized() / count);
            }
            count++;
            lastDouble = aDouble;
        })));
        digitalOutput.set(true);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
        digitalOutput.set(false);
    }
}
