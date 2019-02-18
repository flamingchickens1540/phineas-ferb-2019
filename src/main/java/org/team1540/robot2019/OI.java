package org.team1540.robot2019;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;
import org.apache.log4j.Logger;
import org.team1540.robot2019.commands.cargo.EjectThenDown;
import org.team1540.robot2019.commands.cargo.FloorIntakeCargo;
import org.team1540.robot2019.commands.cargo.LoadingStationIntakeCargo;
import org.team1540.robot2019.commands.climber.ClimbLevelThree;
import org.team1540.robot2019.commands.climber.ClimbLevelTwo;
import org.team1540.robot2019.commands.climber.PrepareForClimb;
import org.team1540.robot2019.commands.climber.ResetClimber;
import org.team1540.robot2019.commands.elevator.MoveElevatorToPosition;
import org.team1540.robot2019.commands.elevator.MoveElevatorToZero;
import org.team1540.robot2019.commands.hatch.GetHatchFloor;
import org.team1540.robot2019.commands.hatch.GrabHatch;
import org.team1540.robot2019.commands.hatch.PlaceHatchThenDown;
import org.team1540.robot2019.commands.hatch.PrepGetPlaceHatch;
import org.team1540.rooster.Utilities;
import org.team1540.rooster.triggers.AxisButton;
import org.team1540.rooster.triggers.DPadAxis;
import org.team1540.rooster.triggers.StrictDPadButton;
import org.team1540.rooster.util.SimpleCommand;

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
    public static final int LEFT_STICK_PRESS = 9;
    public static final int RIGHT_STICK_PRESS = 10;

    // Axes
    public static final int LEFT_X = 0;
    public static final int LEFT_Y = 1;
    public static final int LEFT_TRIG = 2;
    public static final int RIGHT_TRIG = 3;
    public static final int RIGHT_X = 4;
    public static final int RIGHT_Y = 5;

    // Joysticks
    private static XboxController driver = new XboxController(0);
    private static XboxController copilot = new XboxController(1);

    // copilot buttons

    private static Button elevatorMidRocketButton = new StrictDPadButton(copilot, 0, DPadAxis.UP);
    private static Button elevatorCargoShipButton = new StrictDPadButton(copilot, 0, DPadAxis.LEFT);
    private static Button elevatorDownButton = new StrictDPadButton(copilot, 0, DPadAxis.DOWN);
    private static Button intakeLoadingStationButton = new StrictDPadButton(copilot, 0, DPadAxis.RIGHT);

    private static JoystickButton autoIntakeButton = new JoystickButton(copilot, A);
    private static Button cancelIntakeButton = new AxisButton(copilot, Tuning.axisButtonThreshold, RIGHT_Y);
    private static JoystickButton ejectButton = new JoystickButton(copilot, B);

    private static JoystickButton getHatchButton = new JoystickButton(copilot, X);
    private static JoystickButton getHatchFloorButton = new JoystickButton(copilot, START);
    private static Button grabHatchButton = new AxisButton(copilot, Tuning.axisButtonThreshold, RIGHT_TRIG);
    private static JoystickButton placeHatchButton = new JoystickButton(copilot, Y);

    private static JoystickButton prepareToClimbButton = new JoystickButton(copilot, BACK);
    private static JoystickButton startClimbingButton = new JoystickButton(copilot, RB);
    private static AxisButton startClimbingSafety = new AxisButton(copilot, Tuning.axisButtonThreshold, LEFT_TRIG);
    private static JoystickButton climberCylinderUp = new JoystickButton(copilot, LB);
    public static JoystickButton climberResetButton = new JoystickButton(copilot, LEFT_STICK_PRESS);

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
        elevatorCargoShipButton
            .whenPressed(new MoveElevatorToPosition(Tuning.elevatorCargoShipPosition));
        elevatorDownButton.whenPressed(new MoveElevatorToZero());
        intakeLoadingStationButton.whenPressed(new LoadingStationIntakeCargo());

        Command intakeCommand = new FloorIntakeCargo();
        autoIntakeButton.whenPressed(intakeCommand);
        cancelIntakeButton
            .whenPressed(new SimpleCommand("Cancel CargoMech", intakeCommand::cancel));
        ejectButton.whenPressed(new EjectThenDown());

        getHatchButton.whenPressed(new PrepGetPlaceHatch());
        getHatchFloorButton.whenPressed(new GetHatchFloor());
        grabHatchButton.whenPressed(new GrabHatch()); // TODO: Carry hatches with the hatch mechanism in (for vision)
        placeHatchButton.whenPressed(new PlaceHatchThenDown());

        prepareToClimbButton.whenPressed(new PrepareForClimb());
        startClimbingButton.whenPressed(new SimpleCommand("Climb Safe", () -> {
            if (startClimbingSafety.get()) {
                new ClimbLevelThree().start();
            }
        }));
        climberCylinderUp
            .whenPressed(new SimpleCommand("Raise Cylinder", Robot.climber::cylinderUp, Robot.climber));
        climberResetButton.whenPressed(new ResetClimber());
        climberCylinderUp.whenPressed(new SimpleCommand("Foo", () -> {
            if (startClimbingSafety.get()) {
                new ClimbLevelTwo().start();
            }
        }));

        double end = RobotController.getFPGATime() / 1000.0;
        logger.info("Initialized buttons in " + (end - start) + " ms");
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

    public static boolean getDriveFine() {
        return fineDriveButton.get();
    }
}
