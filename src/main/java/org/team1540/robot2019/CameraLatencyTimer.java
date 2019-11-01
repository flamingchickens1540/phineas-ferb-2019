package org.team1540.robot2019;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.command.Command;
import java.util.ArrayList;
import java.util.List;

public class CameraLatencyTimer extends Command {

    private final DigitalOutput digitalOutput;
    private static final List<Integer> callbackIds = new ArrayList<>();
    private static final List<Double> rateTimes = new ArrayList<>();

    public CameraLatencyTimer(int ledChannel) {
        digitalOutput = new DigitalOutput(ledChannel);
    }

    @Override
    protected void initialize() {
        for (int i = 0; i < 20; i++) {
            System.out.print("Timer started!! ");
        }
        if (!Hardware.limelight.isTargetFound()) {
            callbackIds.add(Hardware.limelight.registerCallback((entryNotification -> {
                if (entryNotification.value.getDouble() > 0.5) {
                    double v = this.timeSinceInitialized();
                    rateTimes.add(v);
                    rateTimes.sort(Double::compareTo);
                    callbackIds.stream().forEach(Hardware.limelight::removeCallback);
                    digitalOutput.set(false);
                    double median = rateTimes.get(Math.max(Math.min(rateTimes.size() / 2, rateTimes.size()), 0));
                    double min = rateTimes.get(0);
                    for (int i = 0; i < 200; i++) {
                        System.out.println("Latency measurement: " + median + " this measurement " + v + " min " + min);
                    }
                }
                this.cancel();
            })));
        } else {
            this.cancel();
        }
        digitalOutput.set(true);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
        digitalOutput.set(false);
        this.start();
    }
}
