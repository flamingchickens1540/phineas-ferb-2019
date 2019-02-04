package org.team1540.robot2019;

import static org.team1540.rooster.Utilities.scale;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import org.apache.log4j.Logger;
import org.team1540.robot2019.commands.elevator.MoveElevatorToPosition;
import org.team1540.robot2019.commands.elevator.MoveElevatorToZero;
import org.team1540.robot2019.commands.groups.EjectThenDown;
import org.team1540.robot2019.commands.groups.IntakeSequence;
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
  private static XboxController driver = new XboxController(0);
  ;
  private static XboxController copilot = new XboxController(1);

  // copilot buttons
  private static JoystickButton elevatorMidRocketButton = new JoystickButton(copilot, X);
  private static JoystickButton elevatorCargoShipButton = new JoystickButton(copilot, B);
  private static JoystickButton elevatorDownButton = new JoystickButton(copilot, A);

  private static JoystickButton autoIntakeButton = new JoystickButton(copilot, LB);
  private static JoystickButton ejectButton = new JoystickButton(copilot, RB);

  private static JoystickButton getHatchButton = new JoystickButton(copilot, START);
  private static JoystickButton getHatchFloorButton = new JoystickButton(copilot, START);
  private static JoystickButton placeHatchButton = new JoystickButton(copilot, START);

  private static JoystickButton startClimbingButton = new JoystickButton(copilot, 0);


  /**
   * Since we want to initialize stuff once the robot actually boots up (not as static
   * initializers), we instantiate stuff here to get more informative error traces and less general
   * weirdness.
   */
  static void init() {
    logger.info("Initializing operator interface...");
    double start = RobotController.getFPGATime() / 1000.0; // getFPGATime returns microseconds

    initJoysticks();
    initButtons();

    double end = RobotController.getFPGATime() / 1000.0;
    logger.info("Initialized operator interface in " + (end - start) + " ms");
  }

  public static void initJoysticks() {
    logger.info("Initializing joysticks...");
    double start = RobotController.getFPGATime() / 1000.0; // getFPGATime returns microseconds

    double end = RobotController.getFPGATime() / 1000.0;
    logger.info("Initialized joysticks in " + (end - start) + " ms");
  }

  public static void initButtons() {
    logger.info("Initializing buttons...");
    double start = RobotController.getFPGATime() / 1000.0; // getFPGATime returns microseconds

    elevatorMidRocketButton.whenPressed(new MoveElevatorToPosition(28));
    elevatorCargoShipButton
        .whenPressed(new MoveElevatorToPosition(14));
    elevatorDownButton.whenPressed(new MoveElevatorToZero());

    autoIntakeButton.whenPressed(new IntakeSequence());
    ejectButton.whenPressed(new EjectThenDown());

    // hatch stuff
    // climber stuff

    double end = RobotController.getFPGATime() / 1000.0;
    logger.info("Initialized buttons in " + (end - start) + " ms");
  }

  public static double getDriveThrottle() {
    return scale(
        -Utilities.processDeadzone(driver.getY(GenericHID.Hand.kLeft), Tuning.driveDeadzone),
        Tuning.driveThrottleExponent);
  }

  public static double getDriveSoftTurn() {
    return scale(
        Utilities.processDeadzone(driver.getX(Hand.kRight), Tuning.driveDeadzone),
        Tuning.driveSoftTurnExponent);
  }

  public static double getDriveHardTurn() {
    return scale(
        Utilities.processDeadzone(driver.getTriggerAxis(GenericHID.Hand.kRight), 0.1)
            - Utilities.processDeadzone(driver.getTriggerAxis(GenericHID.Hand.kLeft), 0.1),
        Tuning.driveHardTurnExponent);
  }

  // DRIVETRAIN
  public static double getTankdriveLeftAxis() {
    return scale(Utilities.processDeadzone(driver.getRawAxis(LEFT_Y), Tuning.axisDeadzone), 2);
  }

  public static double getTankdriveRightAxis() {
    return scale(Utilities.processDeadzone(driver.getRawAxis(RIGHT_Y), Tuning.axisDeadzone), 2);
  }

  public static double getTankdriveBackwardsAxis() {
    return scale(Utilities.processDeadzone(driver.getRawAxis(LEFT_TRIG), Tuning.axisDeadzone), 2);
  }

  public static double getTankdriveForwardsAxis() {
    return scale(Utilities.processDeadzone(driver.getRawAxis(RIGHT_TRIG), Tuning.axisDeadzone), 2);
  }
}
