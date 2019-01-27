package org.team1540.robot2019;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import org.team1540.rooster.Utilities;
import sun.nio.ch.Util;

public class OI {

  // Buttons
  public static final int A = 1;
  public static final int B = 2;
  public static final int X = 3;
  public static final int Y = 4;

  public static final int LB = 5;
  public static final int RB = 6;
  public static final int BACK = 7;
  public static final int START = 8;

  // Joysticks
  public static final int LEFT_X = 0;
  public static final int LEFT_Y = 1;
  public static final int LEFT_TRIG = 2;
  public static final int RIGHT_TRIG = 3;
  public static final int RIGHT_X = 4;
  public static final int RIGHT_Y = 5;


  private static XboxController driver;
  private static XboxController copilot;

  // copilot buttons
  private static JoystickButton elevatorMidRocketButton; // initialize them here too
  private static JoystickButton elevatorCargoShipButton;
  private static JoystickButton elevatorDownButton;

  private static JoystickButton autoIntakeButton;
  private static JoystickButton ejectCargoButton;

  private static JoystickButton getHatchButton;
  private static JoystickButton getHatchFloorButton;
  private static JoystickButton placeHatchButton;

  //for climber testing
  public static double getClimberArmJoystick() {
    return Utilities.processDeadzone(copilot.getRawAxis(RIGHT_Y), 0.1);
  }

  /**
   * Since we want to initialize stuff once the robot actually boots up (not as static
   * initializers), we instantiate stuff here to get more informative error traces and less general
   * weirdness.
   */
  static void init() {
    System.out.println("Initializing operator interface...");
    double start = RobotController.getFPGATime() / 1000.0; // getFPGATime returns microseconds

    initJoysticks();

    double end = RobotController.getFPGATime() / 1000.0;
    System.out.println("Initialized operator interface in " + (end - start) + " ms");
  }

  public static void initJoysticks() {
    driver = new XboxController(0);
    copilot = new XboxController(1);
  }

  public static void initButtons() {

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
