package org.team1540.robot2019;

import com.ctre.phoenix.motorcontrol.StickyFaults;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import org.jetbrains.annotations.NotNull;
import org.team1540.rooster.wrappers.ChickenController;

public class PhineasUtilities {

    public static void processStickyFaults(String subsystemName, String motorName,
        @NotNull ChickenController controller) {
        var stickyFaults = new StickyFaults();
        controller.getStickyFaults(stickyFaults);
        if (stickyFaults.hasAnyFault()) {
            var stringBuilder = new StringBuilder()
                .append(stickyFaults.ForwardLimitSwitch ? "fwd lim switch, " : "")
                .append(stickyFaults.ReverseLimitSwitch ? "rev lim switch, " : "")
                .append(stickyFaults.ForwardSoftLimit ? "fwd soft limit, " : "")
                .append(stickyFaults.ReverseSoftLimit ? "rev soft limit, " : "")
                .append(stickyFaults.HardwareESDReset ? "hardware ESD reset, " : "")
                .append(stickyFaults.RemoteLossOfSignal ? "remote loss of signal, " : "")
                .append(stickyFaults.ResetDuringEn ? "reset during enable, " : "")
                .append(stickyFaults.SensorOutOfPhase ? "sensor out of phase, " : "")
                .append(stickyFaults.SensorOverflow ? "sensor overflow, " : "")
                .append(stickyFaults.UnderVoltage ? "undervoltage, " : "");

            if (stringBuilder.length() != 0) {
                // delete the last comma
                stringBuilder.delete(stringBuilder.length() - 3, stringBuilder.length());
            }

            String description =
                subsystemName + " controller " + controller.getDeviceID() + "(" + motorName
                    + ") had sticky faults: " + stringBuilder.toString();
            DriverStation.reportWarning(description, false);

            Shuffleboard
                .addEventMarker(subsystemName + " sticky fault", description, EventImportance.kHigh);
        }
    }

}
