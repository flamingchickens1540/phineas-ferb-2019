package org.team1540.robot2019;

import static org.team1540.rooster.Utilities.scale;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;
import org.apache.log4j.Logger;
import org.team1540.robot2019.commands.cargo.EjectThenDown;
import org.team1540.robot2019.commands.cargo.FloorIntake;
import org.team1540.robot2019.commands.cargo.LoadingStationIntake;
import org.team1540.robot2019.commands.climber.ClimbLevelThree;
import org.team1540.robot2019.commands.climber.ClimbLevelTwo;
import org.team1540.robot2019.commands.elevator.MoveElevatorToPosition;
import org.team1540.robot2019.commands.elevator.MoveElevatorToZero;
import org.team1540.robot2019.commands.hatch.ExtendHatch;
import org.team1540.robot2019.commands.hatch.GetHatchFloor;
import org.team1540.robot2019.commands.hatch.GrabHatch;
import org.team1540.robot2019.commands.hatch.PlaceHatchThenDown;
import org.team1540.rooster.Utilities;
import org.team1540.rooster.triggers.AxisButton;
import org.team1540.rooster.triggers.DPadAxis;
import org.team1540.rooster.triggers.StrictDPadButton;
import org.team1540.rooster.util.SimpleCommand;
import org.team1540.rooster.util.SimpleConditionalCommand;

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
    public static XboxController driver = new XboxController(0);
    public static XboxController copilot = new XboxController(1);

    // copilot buttons

    private static Button elevatorMidRocketButton = new StrictDPadButton(copilot, 0, DPadAxis.UP);
    private static Button elevatorCargoShipButton = new StrictDPadButton(copilot, 0, DPadAxis.LEFT);
    private static Button elevatorDownButton = new StrictDPadButton(copilot, 0, DPadAxis.DOWN);
    private static Button intakeLoadingStationButton = new StrictDPadButton(copilot, 0, DPadAxis.RIGHT);

    private static JoystickButton autoIntakeButton = new JoystickButton(copilot, A);
    private static Button cancelIntakeButton = new AxisButton(copilot, Tuning.axisButtonThreshold, RIGHT_Y);
    private static JoystickButton ejectButton = new JoystickButton(copilot, B);

    private static JoystickButton prepGetHatchButton = new JoystickButton(copilot, X);
    private static JoystickButton prepGetHatchFloorButton = new JoystickButton(copilot, START);
    private static Button grabHatchButton = new AxisButton(copilot, Tuning.axisButtonThreshold, RIGHT_TRIG);
    private static JoystickButton placeHatchButton = new JoystickButton(copilot, Y);

    private static Button climbingSafety = new AxisButton(copilot, Tuning.axisButtonThreshold, LEFT_TRIG);
    private static JoystickButton climbLevel3Button = new JoystickButton(copilot, RB); // + safety
    private static JoystickButton climbLevel2Button = new JoystickButton(copilot, LB); // + safety
    private static JoystickButton climberCylinderUp = new JoystickButton(copilot, LB);// also use this for end of lvl 2 climb

    // driver buttons

    public static JoystickButton fineDriveButton = new JoystickButton(driver, LB);

    /**
     * Since we want to initialize stuff once the robot actually boots up (not as static initializers), we instantiate stuff here to get more informative error traces and less general weirdness.
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

        elevatorMidRocketButton.whenPressed(new MoveElevatorToPosition(Tuning.elevatorUpPosition));
        elevatorCargoShipButton.whenPressed(new MoveElevatorToPosition(Tuning.elevatorCargoShipPosition));
        elevatorDownButton.whenPressed(new MoveElevatorToZero());
        intakeLoadingStationButton.whenPressed(new LoadingStationIntake());

        Command intakeCommand = new FloorIntake();
        autoIntakeButton.whenPressed(new SimpleConditionalCommand(Robot.hatch::noHatch, intakeCommand));
        cancelIntakeButton.whenPressed(new SimpleCommand("Cancel Intake", intakeCommand::cancel));
        ejectButton.whenPressed(new EjectThenDown());

        prepGetHatchButton.whenPressed(new ExtendHatch());
        prepGetHatchFloorButton.whenPressed(new GetHatchFloor());
        grabHatchButton.whenPressed(new GrabHatch());
        placeHatchButton.whenPressed(new PlaceHatchThenDown());

        climbLevel3Button.whenPressed(new SimpleConditionalCommand(climbingSafety::get, new ClimbLevelThree()));
        climbLevel2Button.whenPressed(new SimpleConditionalCommand(climbingSafety::get, new ClimbLevelTwo()));
        climberCylinderUp.whenPressed(new SimpleCommand("Raise Cylinder", Robot.climber::cylinderUp, Robot.climber));

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

    public static boolean getDriveFine() {
        return false;
//        return fineDriveButton.get();
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
