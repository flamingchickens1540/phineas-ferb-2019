package org.team1540.robot2019;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import org.apache.log4j.Logger;
import org.team1540.robot2019.commands.elevator.ZeroElevator;
import org.team1540.rooster.util.SimpleCommand;

public class ShuffleboardDisplay {

  private static final Logger logger = Logger.getLogger(ZeroElevator.class);

  public static void init() {
    logger.info("Initializing Shuffleboard display...");
    double start = RobotController.getFPGATime() / 1000.0; // getFPGATime returns microseconds

    Shuffleboard.getTab("Phineas")
        .add(new SimpleCommand("Reset Preferences", Preferences.getInstance()::removeAll));

    double end = RobotController.getFPGATime() / 1000.0;
    logger.info("Initialized Shuffleboard in " + (end - start) + " ms");
  }

}
