package org.team1540.robot2019;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.apache.log4j.Logger;
import org.team1540.robot2019.commands.auto.PercentManualLineupSequence;
import org.team1540.robot2019.commands.cargo.BackThenDown;
import org.team1540.robot2019.commands.cargo.FloorIntake;
import org.team1540.robot2019.commands.cargo.ForwardThenEject;
import org.team1540.robot2019.commands.cargo.LoadingStationIntake;
import org.team1540.robot2019.commands.climber.ClimbLevelThree;
import org.team1540.robot2019.commands.climber.ClimbLevelTwo;
import org.team1540.robot2019.commands.drivetrain.PointDrive;
import org.team1540.robot2019.commands.drivetrain.TankDriveForTimePercent;
import org.team1540.robot2019.commands.elevator.MoveElevatorToPosition;
import org.team1540.robot2019.commands.elevator.MoveElevatorToZero;
import org.team1540.robot2019.commands.hatch.GrabThenBack;
import org.team1540.robot2019.commands.hatch.PlaceHatchSequence;
import org.team1540.robot2019.commands.hatch.PrepHatchFloorGrab;
import org.team1540.robot2019.commands.hatch.SensorGrabHatchSequence;
import org.team1540.robot2019.commands.hatch.StowHatchMech;
import org.team1540.robot2019.commands.hatch.simple.ExtendHatchMech;
import org.team1540.robot2019.commands.hatch.simple.RetractHatchMech;
import org.team1540.robot2019.commands.leds.BlinkLEDs;
import org.team1540.robot2019.commands.wrist.RecoverWrist;
import org.team1540.robot2019.subsystems.LEDs.LEDColor;
import org.team1540.rooster.Utilities;
import org.team1540.rooster.drive.pipeline.AdvancedArcadeJoystickInput;
import org.team1540.rooster.drive.pipeline.FeedForwardProcessor;
import org.team1540.rooster.drive.pipeline.FeedForwardToVelocityProcessor;
import org.team1540.rooster.triggers.AxisButton;
import org.team1540.rooster.triggers.DPadAxis;
import org.team1540.rooster.triggers.MultiAxisButton;
import org.team1540.rooster.triggers.StrictDPadButton;
import org.team1540.rooster.util.SimpleCommand;
import org.team1540.rooster.util.SimpleConditionalCommand;
import org.team1540.rooster.util.SimpleLoopCommand;

public class OI {

    private static final Logger logger = Logger.getLogger(OI.class);

    // Buttons TODO: Migrate to ROOSTER
    public static final int A = 1;
    public static final int B = 2;
    public static final int X = 3;
    public static final int Y = 4;

    public static final int LB = 5;
    public static final int RB = 6;
    public static final int BACK = 7;
    public static final int START = 8;


    // Axes TODO: Migrate to ROOSTER
    public static final int LEFT_X = 0;
    public static final int LEFT_Y = 1;
    public static final int LEFT_TRIG = 2;
    public static final int RIGHT_TRIG = 3;
    public static final int RIGHT_X = 4;
    public static final int RIGHT_Y = 5;

    // Joysticks
    public static XboxController driver = new XboxController(0);
    private static XboxController copilot = new XboxController(1);

    // Copilot
    // - Elevator
    private static Button elevatorMidRocketButton = new StrictDPadButton(copilot, 0, DPadAxis.UP);
    private static Button elevatorCargoShipButton = new StrictDPadButton(copilot, 0, DPadAxis.LEFT);
    private static Button elevatorDownButton = new StrictDPadButton(copilot, 0, DPadAxis.DOWN);
    private static Button intakeLoadingStationButton = new StrictDPadButton(copilot, 0, DPadAxis.RIGHT);

    // - Intake
    private static JoystickButton autoIntakeButton = new JoystickButton(copilot, A);
    private static Button cancelIntakeButton = new AxisButton(copilot, Tuning.axisButtonThreshold, LEFT_Y);
    private static JoystickButton ejectButton = new JoystickButton(copilot, B);
    private static JoystickButton wristRecoverButton = new JoystickButton(copilot, 9);

    // - Hatch
    private static JoystickButton prepGetHatchButton = new JoystickButton(copilot, X);
    private static JoystickButton prepGetHatchFloorButton = new JoystickButton(copilot, START);
    private static Button grabHatchButton = new AxisButton(copilot, Tuning.axisButtonThreshold, RIGHT_TRIG);
    private static JoystickButton placeHatchButton = new JoystickButton(copilot, Y);
    private static Button stowHatchButton = new AxisButton(copilot, Tuning.axisButtonThreshold, LEFT_TRIG);
    private static Button hatchSimpleForwardButton = new AxisButton(copilot, -Tuning.axisButtonThreshold, LEFT_Y);
    private static Button hatchSimpleBackwardButton = new AxisButton(copilot, Tuning.axisButtonThreshold, LEFT_Y);

    // - Climb
    private static Button climbingSafety = new AxisButton(copilot, Tuning.axisButtonThreshold, LEFT_TRIG);
    private static JoystickButton climbLevel3Button = new JoystickButton(copilot, RB); // + safety
    private static JoystickButton climbLevel2Button = new JoystickButton(copilot, LB); // + safety
    private static JoystickButton climberCylinderUp = new JoystickButton(copilot, BACK);

    // Driver
    // - Auto-align
    public static PercentManualLineupSequence alignCommand;
    public static AxisButton highTargetButton = new AxisButton(driver, 0.5, LEFT_TRIG);
    private static Button autoAlignButtonAlt = new JoystickButton(driver, RB);

    // - Driving
    static PointDrive pointDriveCommand;
    public static final XboxController POINTDRIVE_CONTROLLER = OI.driver;
    public static final Hand POINTDRIVE_POINT_HAND = Hand.kRight;
    public static MultiAxisButton pointDrivePointAxis = new MultiAxisButton(driver, 0.4, new int[]{RIGHT_X, RIGHT_Y});
    public static JoystickButton resetPointOffset = new JoystickButton(driver, Y);

    public static JoystickButton arcadeToggle = new JoystickButton(driver, BACK);

    // - LEDs
    public static StrictDPadButton strobeRedBlueButton = new StrictDPadButton(driver, 0, DPadAxis.DOWN);

    /**
     * Since we want to initialize stuff once the robot actually boots up (not as static initializers), we instantiate stuff here to get more informative error traces and less general weirdness.
     */
    static void init() {
        logger.info("Initializing operator interface...");
        double start = RobotController.getFPGATime() / 1000.0; // getFPGATime returns microseconds

        elevatorMidRocketButton.whenPressed(new MoveElevatorToPosition(Tuning.elevatorUpPosition));
        elevatorCargoShipButton.whenPressed(new MoveElevatorToPosition(Tuning.elevatorCargoShipPosition));
        elevatorDownButton.whenPressed(new MoveElevatorToZero());

        Command loadingIntakeCommand = new LoadingStationIntake();
        intakeLoadingStationButton.whenPressed(loadingIntakeCommand);
        cancelIntakeButton.cancelWhenPressed(loadingIntakeCommand);
        cancelIntakeButton.whenPressed(new MoveElevatorToZero());

        Command intakeCommand = new FloorIntake();
//        autoIntakeButton.whenPressed(new SimpleConditionalCommand(Robot.hatch::isReleased, intakeCommand));
        autoIntakeButton.whenPressed(intakeCommand);
        cancelIntakeButton.cancelWhenPressed(intakeCommand);
        ForwardThenEject command = new ForwardThenEject();
        ejectButton.whenPressed(command);
        ejectButton.whenReleased(new BackThenDown());
        ejectButton.whenReleased(new SimpleCommand("", command::cancel));

//        prepGetHatchButton.whenPressed(new SensorGrabHatchSequence());
        prepGetHatchButton.whenPressed(new SensorGrabHatchSequence());
//        prepGetHatchButton.whenPressed(new TestGrabHatch());
//        placeHatchButton.whenPressed(new PlaceHatchSequence());
        placeHatchButton.whenPressed(new PlaceHatchSequence());
        wristRecoverButton.whileHeld(new RecoverWrist());

        prepGetHatchFloorButton.whenPressed(new PrepHatchFloorGrab());
        grabHatchButton.whenPressed(new GrabThenBack());
        stowHatchButton.whenPressed(new StowHatchMech());

        hatchSimpleForwardButton.whenPressed(new ExtendHatchMech());
        hatchSimpleBackwardButton.whenPressed(new RetractHatchMech());

        Command climbCommand3 = new ClimbLevelThree();
        Command climbCommand2 = new ClimbLevelTwo();
        climbLevel3Button.whenPressed(new SimpleConditionalCommand(climbingSafety::get, climbCommand3));
        climbLevel2Button.whenPressed(new SimpleConditionalCommand(climbingSafety::get, climbCommand2));
        climberCylinderUp.whenPressed(new SimpleCommand("Raise Cylinder", Robot.climber::raiseCylinder, Robot.climber));
//        cancelClimbButton.cancelWhenPressed(climbCommand3);
//        cancelClimbButton.cancelWhenPressed(climbCommand2);

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

        pointDriveCommand = new PointDrive();
        pointDrivePointAxis.whileHeld(new SimpleCommand("", () -> {
            if (!arcadeCommand.isRunning() && !(Robot.drivetrain
                .getCurrentCommand() instanceof TankDriveForTimePercent)) {
                pointDriveCommand.start();
            }
        }));
        pointDrivePointAxis.whenReleased(new SimpleCommand("", () -> {
            pointDriveCommand.cancel();
            if (highTargetButton.get()) {
                Robot.limelight.setPipeline(1);
                Robot.deepSpaceVisionTargetLocalization.setPlaneHeight(RobotMap.ROCKET_BALL_TARGET_HEIGHT);
                Robot.limelight.prepForVision();
//                new PercentManualLineup().start();
            } else {
                Robot.limelight.setPipeline(0);
                Robot.deepSpaceVisionTargetLocalization.setPlaneHeight(RobotMap.HATCH_TARGET_HEIGHT);

            }
        }));
        highTargetButton.whenPressed(new SimpleCommand("", () -> {
            if (!pointDriveCommand.isRunning()) {
                Robot.limelight.setPipeline(1);
                Robot.deepSpaceVisionTargetLocalization.setPlaneHeight(RobotMap.ROCKET_BALL_TARGET_HEIGHT);
                Robot.limelight.prepForVision();

//                new PercentManualLineup().start();
            }
        }));
        intakeLoadingStationButton.whenPressed(pointDriveCommand);
        elevatorCargoShipButton.whenPressed(pointDriveCommand);
        elevatorMidRocketButton.whenPressed(pointDriveCommand);

        Command pmuCommand = new PercentManualLineupSequence();
        pointDrivePointAxis.whenReleased(new SimpleCommand("", () -> {
            if (!arcadeCommand.isRunning()) {
                pmuCommand.start();
            }
        }));
        autoAlignButtonAlt.whenPressed(new SimpleCommand("", () -> {
            if (!arcadeCommand.isRunning()) {
                pmuCommand.start();
            }
        }));
        SimpleCommand resetPointOffset = new SimpleCommand("Reset Point Offset", () -> {
            logger.debug("Setting Angle Offset");
            PointDrive.setInitAngleOffset(Hardware.navx.getYawRadians());
        });
        resetPointOffset.setRunWhenDisabled(true);
        OI.resetPointOffset.whenPressed(resetPointOffset);

        strobeRedBlueButton.whileHeld(new BlinkLEDs(LEDColor.PURPLE, LEDColor.OFF, Tuning.ledStrobeTime));

        double end = RobotController.getFPGATime() / 1000.0;
        logger.info("Initialized operator interface in " + (end - start) + " ms");
    }

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

    public static double getManualClimberArmsAxis() {
        return Utilities.processDeadzone(-copilot.getY(Hand.kRight), Tuning.driveDeadzone);
    }

    // DRIVETRAIN
    public static double getTankdriveLeftAxis() {
        return Utilities
            .scale(Utilities.processDeadzone(driver.getY(Hand.kLeft), Tuning.driveDeadzone), 2);
    }

    public static double getTankdriveRightAxis() {
        return Utilities
            .scale(Utilities.processDeadzone(driver.getY(Hand.kRight), Tuning.driveDeadzone), 2);
    }

    public static double getTankdriveBackwardsAxis() {
        return Utilities.scale(
            Utilities.processDeadzone(driver.getTriggerAxis(Hand.kLeft), Tuning.driveDeadzone), 2);
    }

    public static double getTankdriveForwardsAxis() {
        return Utilities.scale(
            Utilities.processDeadzone(driver.getTriggerAxis(Hand.kRight), Tuning.driveDeadzone), 2);
    }

    public static double getJoystickAngle(XboxController controller, Hand hand) { // TODO: Migrate to ROOSTER
        double x = -controller.getY(hand);
        double y = -controller.getX(hand);
        return Math.atan2(y, x);
    }

    public static double get2DJoystickMagnitude(XboxController controller, Hand hand) { // TODO: Migrate to ROOSTER
        double x = controller.getX(hand);
        double y = controller.getY(hand);
        return Utilities.processDeadzone(new Vector2D(x, y).distance(Vector2D.ZERO), Tuning.driveDeadzone);
    }
}
