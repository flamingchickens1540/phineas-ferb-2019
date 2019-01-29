package org.team1540.robot2019;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import org.apache.log4j.Logger;
import org.team1540.robot2019.commands.gamepieceGroups.EjectThenDown;
import org.team1540.robot2019.commands.gamepieceGroups.IntakeSequence;
import org.team1540.robot2019.commands.gamepieceGroups.MoveElevatorToPosition;
import org.team1540.rooster.Utilities;

public class OI {

  private static final Logger logger = Logger.getLogger(OI.class);

  // Buttons
  public static final int A = 1;
  public static final int B = 2;
  public static final int X = 3;
  public static final int Y = 4;

  public static final int LB = 5;
  public static final int RB = 6;
  public static final int BACK = 7;
  public static final int START = 8;

  // Axes
  public static final int LEFT_X = 0;
  public static final int LEFT_Y = 1;
  public static final int LEFT_TRIG = 2;
  public static final int RIGHT_TRIG = 3;
  public static final int RIGHT_X = 4;
  public static final int RIGHT_Y = 5;

  // Joysticks
  private static XboxController driver;
  private static XboxController copilot;

  // copilot buttons
  private static JoystickButton elevatorMidRocketButton = new JoystickButton(copilot, 0);
  private static JoystickButton elevatorCargoShipButton = new JoystickButton(copilot, 0);
  private static JoystickButton elevatorDownButton = new JoystickButton(copilot, 0);

  private static JoystickButton autoIntakeButton = new JoystickButton(copilot, 0);
  private static JoystickButton ejectButton = new JoystickButton(copilot, 0);

  private static JoystickButton getHatchButton = new JoystickButton(copilot, 0);
  private static JoystickButton getHatchFloorButton = new JoystickButton(copilot, 0);
  private static JoystickButton placeHatchButton = new JoystickButton(copilot, 0);

  // climber stuff

  // for climber testing
  public static double getClimberArmJoystick() {
    return Utilities.processDeadzone(copilot.getY(Hand.kRight), 0.1);
  }

  /**
   * Since we want to initialize stuff once the robot actually boots up (not as static
   * initializers), we instantiate stuff here to get more informative error traces and less general
   * weirdness.
   */
  static void init() {
    logger.info("Initializing operator interface...");
    double start = RobotController.getFPGATime() / 1000.0; // getFPGATime returns microseconds

    initJoysticks();

    double end = RobotController.getFPGATime() / 1000.0;
    logger.info("Initialized operator interface in " + (end - start) + " ms");
  }

  public static void initJoysticks() {
    driver = new XboxController(0);
    copilot = new XboxController(1);
  }

  public static void initButtons() {
    elevatorMidRocketButton.whenPressed(new MoveElevatorToPosition(Tuning.elevatorUpPosition));
    elevatorCargoShipButton.whenPressed(new MoveElevatorToPosition(Tuning.elevatorCargoShipPosition));
    elevatorDownButton.whenPressed(new MoveElevatorToPosition(Tuning.elevatorDownPosition));

    autoIntakeButton.whenPressed(new IntakeSequence());
    ejectButton.whenPressed(new EjectThenDown());

    // hatch stuff
    // climber stuff
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
