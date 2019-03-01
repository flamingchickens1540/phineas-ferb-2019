package org.team1540.robot2019.logging;

import edu.wpi.first.wpilibj.DriverStation;
import org.apache.log4j.AppenderSkeleton;
import org.apache.log4j.Level;
import org.apache.log4j.spi.LoggingEvent;

public class DriverStationAppender extends AppenderSkeleton {

    @Override
    protected void append(LoggingEvent loggingEvent) {
        if (loggingEvent.getLevel().isGreaterOrEqual(Level.WARN)) {
            if (loggingEvent.getLevel().isGreaterOrEqual(Level.ERROR)) {
                DriverStation.reportError(layout.format(loggingEvent), false);
            } else {
                DriverStation.reportWarning(layout.format(loggingEvent), false);
            }
        }
    }

    @Override
    public void close() {

    }

    @Override
    public boolean requiresLayout() {
        return true;
    }
}
