package org.team1540.robot2019;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.command.Command;
import org.apache.log4j.Logger;
import org.team1540.robot2019.commands.cargo.BackThenDown;
import org.team1540.robot2019.commands.cargo.FloorCargoIntake;
import org.team1540.robot2019.commands.cargo.ForwardThenEjectCargo;
import org.team1540.robot2019.commands.cargo.LoadingStationCargoIntake;
import org.team1540.robot2019.commands.climber.ClimbLevelThree;
import org.team1540.robot2019.commands.climber.ClimbLevelTwo;
import org.team1540.robot2019.commands.drivetrain.PointDriveAngleProvider;
import org.team1540.robot2019.commands.elevator.MoveElevatorToPosition;
import org.team1540.robot2019.commands.elevator.MoveElevatorToZero;
import org.team1540.robot2019.commands.hatch.GrabThenRetract;
import org.team1540.robot2019.commands.hatch.PlaceHatchSequence;
import org.team1540.robot2019.commands.hatch.PrepHatchFloorGrab;
import org.team1540.robot2019.commands.hatch.SensorGrabHatchSequence;
import org.team1540.robot2019.commands.hatch.StowHatchMech;
import org.team1540.robot2019.commands.hatch.simple.ExtendHatchMech;
import org.team1540.robot2019.commands.hatch.simple.RetractHatchMech;
import org.team1540.robot2019.commands.leds.BlinkLEDs;
import org.team1540.robot2019.commands.wrist.RecoverWrist;
import org.team1540.robot2019.subsystems.LEDs.LEDColor;
import org.team1540.robot2019.utils.ChickenXboxController;
import org.team1540.robot2019.utils.ChickenXboxController.XboxAxis;
import org.team1540.robot2019.utils.ChickenXboxController.XboxButton;
import org.team1540.rooster.Utilities;
import org.team1540.rooster.drive.pipeline.AdvancedArcadeJoystickInput;
import org.team1540.rooster.drive.pipeline.FeedForwardProcessor;
import org.team1540.rooster.drive.pipeline.FeedForwardToVelocityProcessor;
import org.team1540.rooster.triggers.DPadAxis;
import org.team1540.rooster.util.SimpleCommand;
import org.team1540.rooster.util.SimpleConditionalCommand;
import org.team1540.rooster.util.SimpleLoopCommand;

public class OI {

    private static final Logger logger = Logger.getLogger(OI.class);

    // Joysticks
    private static ChickenXboxController driver = new ChickenXboxController(0);
    private static ChickenXboxController copilot = new ChickenXboxController(1);

    // Copilot
    // - Elevator
    private static Button elevatorFullUpButton = copilot.getButton(DPadAxis.UP); // TODO: ChickenButton
    private static Button elevatorCargoShipButton = copilot.getButton(DPadAxis.LEFT);
    private static Button elevatorDownButton = copilot.getButton(DPadAxis.DOWN);
    private static Button intakeLoadingStationButton = copilot.getButton(DPadAxis.RIGHT);

    // - Intake
    private static Button floorIntakeButton = copilot.getButton(XboxButton.A);
    private static Button cancelIntakeButton = copilot.getButton(XboxAxis.LEFT_Y, Tuning.axisButtonThreshold);
    private static Button ejectButton = copilot.getButton(XboxButton.B);
    private static Button wristRecoverButton = copilot.getButton(XboxButton.LEFT_PRESS);

    // - Hatch
    private static Button prepGetHatchButton = copilot.getButton(XboxButton.X);
    private static Button prepGetHatchFloorButton = copilot.getButton(XboxButton.START);
    private static Button grabHatchButton = copilot.getButton(XboxAxis.RIGHT_TRIG, Tuning.axisButtonThreshold);
    private static Button placeHatchButton = copilot.getButton(XboxButton.Y);
    private static Button stowHatchButton = copilot.getButton(XboxAxis.LEFT_TRIG, Tuning.axisButtonThreshold);
    private static Button hatchSimpleForwardButton = copilot.getButton(XboxAxis.LEFT_Y, -Tuning.axisButtonThreshold);
    private static Button hatchSimpleBackwardButton = copilot.getButton(XboxAxis.LEFT_Y, Tuning.axisButtonThreshold);

    // - Climb
    private static Button climbingSafety = copilot.getButton(XboxAxis.LEFT_TRIG, Tuning.axisButtonThreshold);
    private static Button climbLevel3Button = copilot.getButton(XboxButton.RB); // + safety
    private static Button climbLevel2Button = copilot.getButton(XboxButton.LB); // + safety
    private static Button climberCylinderUp = copilot.getButton(XboxButton.BACK);

    // Driver
    // - Auto-align
    private static Button highTargetButton = driver.getButton(XboxAxis.LEFT_TRIG, 0.5);
    private static Button autoAlignButtonAlt = driver.getButton(XboxButton.RB);

    // - Driving
    private static Button pointDrivePointAxis = driver.getButton(0.4, XboxAxis.RIGHT_X, XboxAxis.RIGHT_Y);
    private static Button resetPointOffset = driver.getButton(XboxButton.Y);

    private static Button arcadeToggle = driver.getButton(XboxButton.BACK);

    // - LEDs
    private static Button strobeRedBlueButton = driver.getButton(DPadAxis.DOWN);

    // - Temporary
    private static Button testPrepGetHatchButton = driver.getButton(XboxButton.A);
    private static Button testPlaceHatchButton = driver.getButton(XboxButton.B);

    private static Button testElevatorFullUpButton = driver.getButton(DPadAxis.UP); // TODO: ChickenButton
    private static Button testElevatorCargoShipButton = driver.getButton(DPadAxis.LEFT);
    private static Button testElevatorDownButton = driver.getButton(DPadAxis.DOWN);
    private static Button testIntakeLoadingStationButton = driver.getButton(DPadAxis.RIGHT);

    /**
     * Since we want to initialize stuff once the robot actually boots up (not as static initializers), we instantiate stuff here to get more informative error traces and less general weirdness.
     */
    static void init() {
        logger.info("Initializing operator interface...");
        double start = RobotController.getFPGATime() / 1000.0; // getFPGATime returns microseconds

        // Elevator
        elevatorFullUpButton.whenPressed(new MoveElevatorToPosition(Tuning.elevatorUpPosition));
        elevatorCargoShipButton.whenPressed(new MoveElevatorToPosition(Tuning.elevatorCargoShipPosition));
        elevatorDownButton.whenPressed(new MoveElevatorToZero());
        intakeLoadingStationButton.whenPressed(new LoadingStationCargoIntake());

        // Intake cargo
        Command floorIntakeCommand = new FloorCargoIntake();
        floorIntakeButton.whenPressed(floorIntakeCommand);
        cancelIntakeButton.cancelWhenPressed(floorIntakeCommand);
        cancelIntakeButton.whenPressed(new MoveElevatorToZero());

        // Wrist
        wristRecoverButton.whileHeld(new RecoverWrist());

        // Eject cargo
        ejectButton.whileHeld(new ForwardThenEjectCargo());
        ejectButton.whenReleased(new BackThenDown());

        // Hatch
        prepGetHatchButton.whenPressed(new SensorGrabHatchSequence());
        placeHatchButton.whenPressed(new PlaceHatchSequence());

        grabHatchButton.whenPressed(new GrabThenRetract());
        stowHatchButton.whenPressed(new StowHatchMech());

        hatchSimpleForwardButton.whenPressed(new ExtendHatchMech());
        hatchSimpleBackwardButton.whenPressed(new RetractHatchMech());

        prepGetHatchFloorButton.whenPressed(new PrepHatchFloorGrab());

        // Temporary
        testPrepGetHatchButton.whenPressed(new SensorGrabHatchSequence());
        testPlaceHatchButton.whenPressed(new PlaceHatchSequence());

        testElevatorFullUpButton.whenPressed(new MoveElevatorToPosition(Tuning.elevatorUpPosition));
        testElevatorFullUpButton.whenPressed(new SimpleCommand("", Robot.drivetrain.getDriveCommand()::tempDisableLineup));

        testElevatorDownButton.whenPressed(new MoveElevatorToZero());

        // Climb
        climbLevel3Button.whenPressed(new SimpleConditionalCommand(climbingSafety::get, new ClimbLevelThree()));
        climbLevel2Button.whenPressed(new SimpleConditionalCommand(climbingSafety::get, new ClimbLevelTwo()));
        climberCylinderUp.whenPressed(new SimpleCommand("Raise Cylinder", Robot.climber::raiseCylinder, Robot.climber));

        // Arcade drive
        Command arcadeCommand = new SimpleLoopCommand("Drive",
            new AdvancedArcadeJoystickInput(true, OI::getArcadeDriveThrottle, OI::getArcadeDriveSoftTurn,
                OI::getArcadeDriveHardTurn)
                .then(new FeedForwardToVelocityProcessor(Tuning.driveMaxVel))
                .then(new FeedForwardProcessor(Tuning.driveKV, Tuning.driveVIntercept, 0))
                .then(Robot.drivetrain.getPipelineOutput(false)), Robot.drivetrain);
        arcadeToggle.whenPressed(new SimpleCommand("Arcade toggle", () -> {
            if (arcadeCommand.isRunning()) {
                arcadeCommand.cancel();
            } else {
                arcadeCommand.start();
            }
        }));

        // Point drive
        SimpleCommand resetPointOffset = new SimpleCommand("Reset Point Offset", () -> {
            logger.debug("Setting Angle Offset");
            PointDriveAngleProvider.setInitAngleOffset(Hardware.navx.getYawRadians());
        });
        resetPointOffset.setRunWhenDisabled(true);
        OI.resetPointOffset.whenPressed(resetPointOffset);

        // Auto-align cancel
        intakeLoadingStationButton.whenPressed(new SimpleCommand("", Robot.drivetrain.getDriveCommand()::tempDisableLineup));
        elevatorCargoShipButton.whenPressed(new SimpleCommand("", Robot.drivetrain.getDriveCommand()::tempDisableLineup));
        elevatorFullUpButton.whenPressed(new SimpleCommand("", Robot.drivetrain.getDriveCommand()::tempDisableLineup));
        floorIntakeButton.whenPressed(new SimpleCommand("", Robot.drivetrain.getDriveCommand()::tempDisableLineup));

        // High vision target
        highTargetButton.whenPressed(new SimpleCommand("", () -> queueBallRocketTargetFlag = true));

        // Flash LEDs
        strobeRedBlueButton.whileHeld(new BlinkLEDs(LEDColor.PURPLE, LEDColor.OFF, Tuning.ledStrobeTime));

        double end = RobotController.getFPGATime() / 1000.0;
        logger.info("Initialized operator interface in " + (end - start) + " ms");
    }

    // Climber arms
    public static double getManualClimberArmsAxis() {
        return Utilities.processDeadzone(-copilot.getY(Hand.kRight), Tuning.driveDeadzone);
    }

    // Point drive
    public static double getPointDriveThrottle() {
        return -Utilities.scale(Utilities.processDeadzone(driver.getY(Hand.kLeft), Tuning.driveDeadzone), 2);
    }

    public static double getPointDriveAngle() {
        return driver.get2DJoystickAngle(Hand.kRight);
    }

    public static double getPointDriveMagnitude() {
        return Utilities.processDeadzone(driver.get2DJoystickMagnitude(Hand.kRight), Tuning.driveDeadzone);
    }

    // Arcade drive
    public static double getArcadeDriveThrottle() {
        return Utilities.scale(
            -Utilities.processDeadzone(driver.getY(GenericHID.Hand.kLeft), Tuning.driveDeadzone),
            Tuning.driveThrottleExponent);
    }

    public static double getArcadeDriveSoftTurn() {
        return Utilities.scale(
            Utilities.processDeadzone(driver.getX(Hand.kRight), Tuning.driveDeadzone),
            Tuning.driveSoftTurnExponent);
    }

    public static double getArcadeDriveHardTurn() {
        return Utilities.scale(
            Utilities.processDeadzone(driver.getTriggerAxis(GenericHID.Hand.kRight), 0.1)
                - Utilities.processDeadzone(driver.getTriggerAxis(GenericHID.Hand.kLeft), 0.1),
            Tuning.driveHardTurnExponent);
    }

    // Tank drive
    public static double getTankdriveLeftAxis() {
        return Utilities.scale(Utilities.processDeadzone(driver.getY(Hand.kLeft), Tuning.driveDeadzone), 2);
    }

    public static double getTankdriveRightAxis() {
        return Utilities.scale(Utilities.processDeadzone(driver.getY(Hand.kRight), Tuning.driveDeadzone), 2);
    }

    public static double getTankdriveBackwardsAxis() {
        return Utilities.scale(Utilities.processDeadzone(driver.getTriggerAxis(Hand.kLeft), Tuning.driveDeadzone), 2);
    }

    public static double getTankdriveForwardsAxis() {
        return Utilities.scale(Utilities.processDeadzone(driver.getTriggerAxis(Hand.kRight), Tuning.driveDeadzone), 2);
    }

    private static boolean queueBallRocketTargetFlag = false;

    public static boolean clearBallRocketTargetFlag() { // return queueBallRocketTargetFlag != (queueBallRocketTargetFlag = false); ::thonk::
        if (queueBallRocketTargetFlag) {
            queueBallRocketTargetFlag = false;
            return true;
        } else {
            return false;
        }
    }
}
