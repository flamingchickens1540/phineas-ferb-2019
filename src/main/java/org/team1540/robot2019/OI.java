package org.team1540.robot2019;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import org.team1540.rooster.Utilities;

public class OI {

  private static XboxController driver;
  private static XboxController copilot;

  /**
   * Since we want to initialize stuff once the robot actually boots up (not as static
   * initializers), we instantiate stuff here to get more informative error traces and less general
   * weirdness.
   */
  static void init() {
    System.out.println("Initializing operator interface...");
    double start = RobotController.getFPGATime() / 1000.0; // getFPGATime returns microseconds

    driver = new XboxController(0);
    copilot = new XboxController(1);

    double end = RobotController.getFPGATime() / 1000.0;
    System.out.println("Initialized operator interface in " + (end - start) + " ms");
  }

  public static double getDriveThrottle() {
    return Utilities.scale(
        -Utilities.processDeadzone(driver.getY(GenericHID.Hand.kLeft), Tuning.driveDeadzone),
        Tuning.driveThrottleExponent);
  }

  public static double getDriveSoftTurn() {
    return Utilities.scale(
        Utilities.processDeadzone(driver.getX(Hand.kRight), Tuning.driveDeadzone),
        Tuning.driveSoftTurnExponent);
  }

  public static double getDriveHardTurn() {
    return Utilities.scale(
        Utilities.processDeadzone(driver.getTriggerAxis(GenericHID.Hand.kRight), 0.1)
            - Utilities.processDeadzone(driver.getTriggerAxis(GenericHID.Hand.kLeft), 0.1),
        Tuning.driveHardTurnExponent);
  }
}
