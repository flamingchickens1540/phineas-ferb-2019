package org.team1540.robot2019;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import java.util.Map;
import org.apache.log4j.Logger;
import org.team1540.robot2019.commands.selftests.SelfTest;
import org.team1540.rooster.util.SimpleCommand;
import org.team1540.rooster.util.SimpleLoopCommand;

public class ShuffleboardDisplay {

    private static final Logger logger = Logger.getLogger(ShuffleboardDisplay.class);

    private static NetworkTableEntry pressureEntry;
    private static NetworkTableEntry navxConnEntry;
    private static NetworkTableEntry navxCalEntry;
    private static NetworkTableEntry limelightConnEntry;

    public static void init() {
        logger.info("Initializing Shuffleboard display...");
        double start = RobotController.getFPGATime() / 1000.0; // getFPGATime returns microseconds

        ShuffleboardTab tab = Shuffleboard.getTab("Phineas");
        tab.add(new SimpleCommand("Reset Preferences", Preferences.getInstance()::removeAll))
            .withPosition(2, 0);

        pressureEntry = tab.add("System Pressure", 0)
            .withWidget(BuiltInWidgets.kDial)
            .withProperties(Map.of("Min", 0, "Max", 120))
            .withPosition(0, 0)
            .withSize(2, 2)
            .getEntry();

        tab.add(new SelfTest());

        ShuffleboardLayout statusLayout = tab.getLayout("System Status", BuiltInLayouts.kList);

        navxConnEntry = statusLayout.add("NavX Conn", Hardware.navx.isConnected()).getEntry();
        navxCalEntry = statusLayout.add("NavX Cal", Hardware.navx.isCalibrating()).getEntry();

        limelightConnEntry = statusLayout.add("Limelight Conn", false).getEntry();

        // initialize a loop command to update values
        Command command = new SimpleLoopCommand("Shuffleboard Update", ShuffleboardDisplay::update);
        command.setRunWhenDisabled(true);
        command.start();

        double end = RobotController.getFPGATime() / 1000.0;
        logger.info("Initialized Shuffleboard in " + (end - start) + " ms");
    }

    private static void update() {
        pressureEntry.forceSetNumber(50 * (Hardware.pressureSensor.getVoltage() - 0.5));

        navxConnEntry.forceSetBoolean(Hardware.navx.isConnected());
        navxCalEntry.forceSetBoolean(Hardware.navx.isCalibrating());

        limelightConnEntry.forceSetBoolean(Robot.limelight.isConnected());
    }
}
