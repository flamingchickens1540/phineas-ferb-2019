package org.team1540.robot2019;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.command.Command;
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
    public static  ChickenXboxController driver  = new ChickenXboxController(0);
    private static ChickenXboxController copilot = new ChickenXboxController(1);

    // Copilot
    // - Elevator
    private static Button elevatorMidRocketButton    = copilot.getDPadButton(DPadAxis.UP);
    private static Button elevatorCargoShipButton    = copilot.getDPadButton(DPadAxis.LEFT);
    private static Button elevatorDownButton         = copilot.getDPadButton(DPadAxis.DOWN);
    private static Button intakeLoadingStationButton = copilot.getDPadButton(DPadAxis.RIGHT);

    // - Intake
    private static Button autoIntakeButton   = copilot.getButton(XboxButton.A);
    private static Button cancelIntakeButton = copilot.getAxisButton(Tuning.axisButtonThreshold, XboxAxis.LEFT_Y);
    private static Button ejectButton        = copilot.getButton(XboxButton.B);
    private static Button wristRecoverButton = copilot.getButton(XboxButton.LEFT_PRESS);

    // - Hatch
    private static Button prepGetHatchButton        = copilot.getButton(XboxButton.X);
    private static Button prepGetHatchFloorButton   = copilot.getButton(XboxButton.START);
    private static Button grabHatchButton           = copilot.getAxisButton(Tuning.axisButtonThreshold, XboxAxis.RIGHT_TRIG);
    private static Button placeHatchButton          = copilot.getButton(XboxButton.Y);
    private static Button stowHatchButton           = copilot.getAxisButton(Tuning.axisButtonThreshold, XboxAxis.LEFT_TRIG);
    private static Button hatchSimpleForwardButton  = copilot.getAxisButton(-Tuning.axisButtonThreshold, XboxAxis.LEFT_Y);
    private static Button hatchSimpleBackwardButton = copilot.getAxisButton(Tuning.axisButtonThreshold, XboxAxis.LEFT_Y);

    // - Climb
    private static Button climbingSafety    = copilot.getAxisButton(Tuning.axisButtonThreshold, XboxAxis.LEFT_TRIG);
    private static Button climbLevel3Button = copilot.getButton(XboxButton.RB); // + safety
    private static Button climbLevel2Button = copilot.getButton(XboxButton.LB); // + safety
    private static Button climberCylinderUp = copilot.getButton(XboxButton.BACK);

    // Driver
    // - Auto-align
    public static PercentManualLineupSequence alignCommand;
    private static Button highTargetButton = driver.getAxisButton(0.5, XboxAxis.LEFT_TRIG);
    private static Button autoAlignButtonAlt = driver.getButton(XboxButton.RB);

    // - Driving
    static PointDrive pointDriveCommand;

    public static Button pointDrivePointAxis = driver.getMultiAxisButton(0.4, new XboxAxis[]{XboxAxis.RIGHT_X, XboxAxis.RIGHT_Y});
    public static Button resetPointOffset    = driver.getButton(XboxButton.Y);

    public static Button arcadeToggle = driver.getButton(XboxButton.BACK);

    // - LEDs
    public static Button strobeRedBlueButton = driver.getDPadButton(DPadAxis.DOWN);

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

    public static double getPointDriveAngle() {
        return driver.get2DJoystickAngle(Hand.kRight);
    }

    public static double getPointDriveMagnitude() {
        return Utilities.processDeadzone(driver.get2DJoystickMagnitude(Hand.kRight), Tuning.driveDeadzone);
    }
}
