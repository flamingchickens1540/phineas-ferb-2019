package org.team1540.robot2019;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import java.util.Map;
import org.apache.log4j.Logger;
import org.team1540.robot2019.commands.selftests.SelfTest;
import org.team1540.robot2019.utils.ShuffleboardFakeGyro;
import org.team1540.rooster.util.SimpleLoopCommand;

public class PhineasShuffleboardTab {

    private static final Logger logger = Logger.getLogger(PhineasShuffleboardTab.class);

    private static NetworkTableEntry pressureEntry;
    private static ShuffleboardFakeGyro pointDriveFakeGyro = new ShuffleboardFakeGyro();


    public static void init() {
        logger.info("Initializing Shuffleboard display...");
        double start = RobotController.getFPGATime() / 1000.0; // getFPGATime returns microseconds

        ShuffleboardTab tab = Shuffleboard.getTab("Phineas");
//        tab.add(new SimpleCommand("Reset Preferences", Preferences.getInstance()::removeAll))
//            .withPosition(2, 0);

        pressureEntry = tab.add("System Pressure", 0)
            .withWidget(BuiltInWidgets.kDial)
            .withProperties(Map.of("Min", 0, "Max", 120))
            .withPosition(0, 0)
            .withSize(3, 3)
            .getEntry();

        // initialize a loop command to update values
        Command command = new SimpleLoopCommand("Shuffleboard Update", PhineasShuffleboardTab::update);
        command.setRunWhenDisabled(true);
        command.start();

        Shuffleboard.getTab("Phineas")
            .add(new SelfTest());

        tab.add("Point Drive Heading", pointDriveFakeGyro).withProperties(Map.of("Starting angle", 0));

        double end = RobotController.getFPGATime() / 1000.0;
        logger.info("Initialized Shuffleboard in " + (end - start) + " ms");
    }

    private static void update() {
        pressureEntry.forceSetNumber(Hardware.returnPressureSensorValue());
//        pointDriveFakeGyro.setAngle(180 - Math.toDegrees(OI.getPointDriveAngle()));
    }
}
