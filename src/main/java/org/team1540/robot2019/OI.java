package org.team1540.robot2019;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import org.apache.log4j.Logger;
import org.team1540.robot2019.commands.auto.DriveSensorGrabHatchSequence;
import org.team1540.robot2019.commands.auto.TurnUntilNewTarget;
import org.team1540.robot2019.commands.auto.VisionAutoPlaceSequence;
import org.team1540.robot2019.commands.cargo.DriveBackThenElevatorDown;
import org.team1540.robot2019.commands.cargo.FloorIntakeCargo;
import org.team1540.robot2019.commands.cargo.ForwardThenEjectCargo;
import org.team1540.robot2019.commands.cargo.LoadingStationIntakeCargo;
import org.team1540.robot2019.commands.climber.LiftGyroStabilizeLevel2;
import org.team1540.robot2019.commands.climber.LiftGyroStabilizeLevel3Group;
import org.team1540.robot2019.commands.climber.PrepClimbLevelThree;
import org.team1540.robot2019.commands.climber.PrepClimbLevelTwo;
import org.team1540.robot2019.commands.drivetrain.pointdrive.PointDriveAngleProvider;
import org.team1540.robot2019.commands.elevator.MoveElevatorToPosition;
import org.team1540.robot2019.commands.elevator.MoveElevatorToZero;
import org.team1540.robot2019.commands.hatch.PlaceHatchSequence;
import org.team1540.robot2019.commands.hatch.floor.PrepHatchFloorGrab;
import org.team1540.robot2019.commands.hatch.sensor.SensorGrabHatchSequence;
import org.team1540.robot2019.commands.hatch.simple.ReleaseHatch;
import org.team1540.robot2019.commands.hatch.simple.RetractHatchMech;
import org.team1540.robot2019.commands.hatch.subgroups.GrabHatchThenRetract;
import org.team1540.robot2019.commands.hatch.temporary.WiggleAndGrabHatch;
import org.team1540.robot2019.commands.leds.BlinkLEDsAndTurnOffLimelight;
import org.team1540.robot2019.commands.wrist.RecoverWrist;
import org.team1540.robot2019.subsystems.LEDs.LEDColor;
import org.team1540.robot2019.utils.ChickenXboxController;
import org.team1540.robot2019.utils.ChickenXboxController.XboxAxis;
import org.team1540.robot2019.utils.ChickenXboxController.XboxButton;
import org.team1540.robot2019.wrappers.SwitchFilterButton;
import org.team1540.rooster.Utilities;
import org.team1540.rooster.triggers.DPadAxis;
import org.team1540.rooster.util.SimpleCommand;
import org.team1540.rooster.util.SimpleConditionalCommand;

public class OI {

    // TODO: ChickenButton with better logic (whileReleased, whenPressed runnable, whenReleased runnable, etc)

    private static final Logger logger = Logger.getLogger(OI.class);

    private static final boolean INIT_TEMPORARY_BINDINGS = !Tuning.isComp;

    // Joysticks
    private static ChickenXboxController driver = new ChickenXboxController(0);
    private static ChickenXboxController copilot = new ChickenXboxController(1);
    private static ChickenXboxController tester = new ChickenXboxController(2);

    // ---------------------------------------- Copilot ----------------------------------------
    // Hatch
    private static Button sensorGrabHatchButton = copilot.getButton(XboxButton.X);
    private static Button placeHatchSequenceButton = copilot.getButton(XboxButton.START);
    private static Button grabThenRetractButtonAndAlsoTheRocketBallIntakeButton = copilot.getButton(XboxAxis.RIGHT_TRIG, Tuning.axisButtonThreshold);
    private static Button visionPlaceHatchButton = copilot.getButton(XboxButton.Y);
    private static Button stowHatchButton = copilot.getButton(XboxAxis.LEFT_TRIG, Tuning.axisButtonThreshold);

    // Elevator
    private static Button elevatorFullUpButton = copilot.getButton(DPadAxis.UP);
    private static Button elevatorCargoShipButton = copilot.getButton(DPadAxis.LEFT);
    private static Button elevatorDownButton = copilot.getButton(DPadAxis.DOWN);
    private static Button moveElevatorToClimbArmsBackButton = copilot.getButton(XboxButton.RIGHT_PRESS);

    // Cargo
    private static Button cargoFloorIntakeButton = copilot.getButton(XboxButton.A);
    private static Button cargoIntakeLoadingStationButton = copilot.getButton(DPadAxis.RIGHT);

    private static Button cargoEjectButton = copilot.getButton(XboxButton.B);

    private static Button wristRecoverButton = copilot.getButton(XboxAxis.LEFT_Y, Tuning.axisButtonThreshold);

//    private static Button visionPlaceHatchLeft = copilot.getButton(XboxButton.LB);
//    private static Button visionPlaceHatchRight = copilot.getButton(XboxButton.RB);

    // Climb
    private static Button climbingSafety = copilot.getButton(XboxAxis.LEFT_TRIG, Tuning.axisButtonThreshold);
    private static Button climbLevel3Button = copilot.getButton(XboxButton.RB); // + safety
    private static Button prepClimbLevel2Button = copilot.getButton(XboxButton.LB); // + safety
    private static Button climbLevel2Button = copilot.getButton(XboxAxis.LEFT_Y, -Tuning.axisButtonThreshold); // + safety
    private static Button climberCylinderUp = copilot.getButton(XboxButton.BACK);

    // Climber arms
    public static double getManualClimberArmsAxis() {
        return Utilities.processDeadzone(-copilot.getY(Hand.kRight), Tuning.driveDeadzone);
    }

    // ---------------------------------------- Driver ----------------------------------------
    // Auto-align
    private static Button leftFilterButton = driver.getButton(XboxAxis.LEFT_TRIG, 0.3);
    private static Button rightFilterButton = driver.getButton(XboxAxis.RIGHT_TRIG, 0.3);

    // Wiggle wiggle wiggle
    private static Button wiggleButton = driver.getButton(XboxButton.START);

    // Driving
    public static final XboxAxis POINT_DRIVE_POINT_X = XboxAxis.RIGHT_X;
    public static final XboxAxis POINT_DRIVE_POINT_Y = XboxAxis.RIGHT_Y;
    public static final XboxAxis POINT_DRIVE_THROTTLE = XboxAxis.LEFT_Y;

    private static Button resetPointOffset = driver.getButton(XboxButton.Y);

    private static Button lockPointDrive = driver.getButton(XboxButton.RIGHT_PRESS);

    private static Button pointDrivePointAxis = driver.getButton(Tuning.driveDeadzone, POINT_DRIVE_POINT_X, POINT_DRIVE_POINT_Y);
    private static Button pointDriveThrottle = driver.getButton(Tuning.driveDeadzone, POINT_DRIVE_THROTTLE);

    // Point drive
    public static double getPointDriveThrottle() {
        return -Utilities.scale(Utilities.processDeadzone(driver.getRawAxis(POINT_DRIVE_THROTTLE), Tuning.driveDeadzone), 2);
    }

    public static double getPointDriveAngle() {
        return driver.get2DJoystickAngle(Hand.kRight);
    }

    public static double getPointDriveMagnitude() {
        return Utilities.processDeadzone(driver.get2DJoystickMagnitude(Hand.kRight), Tuning.driveDeadzone);
    }

    // LEDs and limelight off
    private static Button turnOffLimelightAndFlashLEDs = driver.getButton(DPadAxis.DOWN);

    // Next left/right target
    private static Button nextLeftTarget = driver.getButton(XboxButton.LB);
    private static Button nextRightTarget = driver.getButton(XboxButton.RB);

    // Temporary
    private static Button testPrepGetHatchButton = driver.getButton(XboxButton.A);
    private static Button testPlaceHatchButton = driver.getButton(XboxButton.B);
    private static Button testPlaceHatchInLoadingStationButton = driver.getButton(XboxButton.X);

    private static Button testElevatorFullUpButton = driver.getButton(DPadAxis.UP);
    private static Button testElevatorDownButton = driver.getButton(DPadAxis.DOWN);

//    private static Button testCargoFloorIntakeButton = driver.getButton(DPadAxis.LEFT);

//    private static Button autoPlaceButton = driver.getButton(XboxButton.B);
//    private static Button autoGrabButton = driver.getButton(XboxButton.A);

    // ---------------------------------------- Tester ----------------------------------------
    private static Button climbSolToggle = tester.getButton(XboxButton.START);
    private static Button hatchSlideSolToggle = tester.getButton(XboxButton.A);
    private static Button hatchGrabSolToggle = tester.getButton(XboxButton.B);

    /**
     * Since we want to initialize stuff once the robot actually boots up (not as static initializers), we instantiate stuff here to get more informative error traces and less general weirdness.
     */
    static void init() {
        logger.info("Initializing operator interface...");
        double start = RobotController.getFPGATime() / 1000.0; // getFPGATime returns microseconds

        // ---------------------------------------- Copilot ----------------------------------------
        // Hatch
        //    Regular hatch sequences
        SensorGrabHatchSequence sensorGrabHatchSequence = new SensorGrabHatchSequence();
        sensorGrabHatchButton.whenPressed(sensorGrabHatchSequence);
        PlaceHatchSequence placeHatchSequence = new PlaceHatchSequence(false, true);
        placeHatchSequenceButton.whenPressed(placeHatchSequence);

        //    Floor hatch grab
        Command prepHatchFloorGrab = new PrepHatchFloorGrab();
        prepClimbLevel2Button.whileHeld(new SimpleCommand("", () -> { // TODO: Replace with simpleButton
            if (climbLevel3Button.get()) {
                prepHatchFloorGrab.start();
            }
        }));
        prepClimbLevel2Button.whenReleased(new SimpleCommand("", prepHatchFloorGrab::cancel));
        climbLevel3Button.whenReleased(new SimpleCommand("", prepHatchFloorGrab::cancel));

        //    Rarely used hatch buttons
        grabThenRetractButtonAndAlsoTheRocketBallIntakeButton.whenPressed(new GrabHatchThenRetract(Tuning.hatchGrabWaitTime));
        stowHatchButton.whenPressed(new CommandGroup() {{
            addSequential(new ReleaseHatch());
            addSequential(new RetractHatchMech());
        }});

        //    Vision hatch place
        VisionAutoPlaceSequence visionAutoPlaceSequence = new VisionAutoPlaceSequence();
        visionPlaceHatchButton.whenPressed(visionAutoPlaceSequence);
        visionPlaceHatchButton.whenReleased(new SimpleCommand("", visionAutoPlaceSequence::cancel));

        // High vision target
        grabThenRetractButtonAndAlsoTheRocketBallIntakeButton.whenPressed(new SimpleCommand("", Robot.drivetrain.getDriveCommand().getLineupLocalization()::enableRocketBallModeForNextCycle));
        grabThenRetractButtonAndAlsoTheRocketBallIntakeButton.whenReleased(new SimpleCommand("", Robot.drivetrain.getDriveCommand().getLineupLocalization()::enableHatchModeForNextCycle));

        // Elevator
        MoveElevatorToPosition moveElevatorUp = new MoveElevatorToPosition(Tuning.elevatorUpPosition);
        elevatorFullUpButton.whenPressed(moveElevatorUp);
        MoveElevatorToPosition moveElevatorToCargoShip = new MoveElevatorToPosition(Tuning.elevatorCargoShipPosition);
        elevatorCargoShipButton.whenPressed(moveElevatorToCargoShip);
        MoveElevatorToZero moveElevatorToZero = new MoveElevatorToZero();
        elevatorDownButton.whenPressed(moveElevatorToZero);
//        MoveElevatorToPosition moveElevatorToClimbArmsBack = new MoveElevatorToPosition(Tuning.elevatorToClimbArmsBack);
//        moveElevatorToClimbArmsBackButton.whenPressed(moveElevatorToClimbArmsBack);

        // Cargo
        Command cargoFloorIntake = new FloorIntakeCargo();
        cargoFloorIntakeButton.whenPressed(cargoFloorIntake);

        cargoIntakeLoadingStationButton.whenPressed(new LoadingStationIntakeCargo());

        // Wrist
        wristRecoverButton.whileHeld(new RecoverWrist());

        // Eject cargo
        ForwardThenEjectCargo forwardThenEjectCargo = new ForwardThenEjectCargo();
        cargoEjectButton.whileHeld(forwardThenEjectCargo);
        DriveBackThenElevatorDown backThenDown = new DriveBackThenElevatorDown();
        cargoEjectButton.whenReleased(backThenDown);

        // Climb
        climbLevel3Button.whenPressed(new SimpleConditionalCommand(climbingSafety::get, new PrepClimbLevelThree()));
        prepClimbLevel2Button.whenPressed(new SimpleCommand("", () -> {
            if (climbingSafety.get()) {
                new PrepClimbLevelTwo().start();
            }
        }));

        LiftGyroStabilizeLevel3Group liftGyroStabilizeLevel3 = new LiftGyroStabilizeLevel3Group();
        climbLevel2Button.whenPressed(new SimpleCommand("", () -> {
            if (PrepClimbLevelTwo.hasPrepLvl2) {
                new LiftGyroStabilizeLevel2().start();
            } else if (PrepClimbLevelThree.hasPrepLvl3) {
                liftGyroStabilizeLevel3.start();
            }
        }));
        climberCylinderUp.whenPressed(new SimpleCommand("Raise Cylinder", Robot.climber::raiseCylinder, Robot.climber));

        // ---------------------------------------- Driver ----------------------------------------

        // Point offset
        SimpleCommand resetPointOffset = new SimpleCommand("Reset Point Offset", () -> {
            logger.debug("Setting Angle Offset");
            PointDriveAngleProvider.setInitAngleOffset(Hardware.navx.getYawRadians());
        });
        resetPointOffset.setRunWhenDisabled(true);
        OI.resetPointOffset.whenPressed(resetPointOffset);

        // Wiggle and grab button
        WiggleAndGrabHatch wiggleAndGrab = new WiggleAndGrabHatch();
        wiggleButton.whenPressed(new SimpleCommand("", () -> {
            boolean running = sensorGrabHatchSequence.isRunning();
            logger.debug("SensorGrabHatchSequence running: " + running);
            if (running) {
                wiggleAndGrab.start();
            }
        }));

        // Interrupt into point drive
        SimpleCommand runPointDrive = new SimpleCommand("", () -> {
            if (Robot.drivetrain.getCurrentCommand() instanceof DriveSensorGrabHatchSequence || Robot.drivetrain.getCurrentCommand() instanceof VisionAutoPlaceSequence) {
                Robot.drivetrain.getDriveCommand().start();
            }
        });
        pointDrivePointAxis.whenPressed(runPointDrive);
        pointDriveThrottle.whenPressed(runPointDrive);

        // Next left/right target
        nextLeftTarget.whenPressed(new TurnUntilNewTarget(Robot.odometry, Robot.deepSpaceVisionTargetLocalization, true));
        nextRightTarget.whenPressed(new TurnUntilNewTarget(Robot.odometry, Robot.deepSpaceVisionTargetLocalization, false));

        // Left/right filtering
        leftFilterButton.whenPressed(new SwitchFilterButton(2));
        leftFilterButton.whenReleased(new SwitchFilterButton(0));
        rightFilterButton.whenPressed(new SwitchFilterButton(3));
        rightFilterButton.whenReleased(new SwitchFilterButton(0));

        // Flash LEDs and turn off limelight
        turnOffLimelightAndFlashLEDs.whileHeld(new BlinkLEDsAndTurnOffLimelight(LEDColor.PURPLE, LEDColor.OFF, Tuning.ledStrobeTime));

        // ---------------------------------------- Temporary ----------------------------------------
        if (INIT_TEMPORARY_BINDINGS) {
            // Hatch
            testPrepGetHatchButton.whenPressed(sensorGrabHatchSequence);
            testPlaceHatchButton.whenPressed(placeHatchSequence);
            testPlaceHatchInLoadingStationButton.whenPressed(new VisionAutoPlaceSequence());
//            testPlaceHatchInLoadingStationButton.whenPressed(new PlaceHatchInLoadingStation());

//            autoPlaceButton.whenPressed(new VisionAutoPlaceSequence());
//            autoGrabButton.whenPressed(new VisionAutoGrabSequence());

            // Elevator
            testElevatorFullUpButton.whenPressed(moveElevatorUp);
            testElevatorDownButton.whenPressed(moveElevatorToZero);

            // Cargo
//            testCargoFloorIntakeButton.toggleWhenPressed(cargoFloorIntake);
//            testBallEjectButton.whileHeld(forwardThenEjectCargo);
//            testBallEjectButton.whenReleased(backThenDown);
        }

        // ---------------------------------------- Tester ----------------------------------------
        climbSolToggle.whenPressed(new SimpleCommand("", () -> {
            if (Robot.climber.isCylLowered()) {
                Robot.climber.raiseCylinder();
            } else {
                Robot.climber.lowerCylinder();
            }
        }));
        hatchSlideSolToggle.whenPressed(new SimpleCommand("", () -> {
            if (Robot.hatch.isExtended()) {
                Robot.hatch.retract();
            } else {
                Robot.hatch.extend();
            }
        }));
        hatchGrabSolToggle.whenPressed(new SimpleCommand("", () -> {
            if (Robot.hatch.isGrabbed()) {
                Robot.hatch.release();
            } else {
                Robot.hatch.grab();
            }
        }));

        double end = RobotController.getFPGATime() / 1000.0;
        logger.info("Initialized operator interface in " + (end - start) + " ms");
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

    public static double getPointUntilNextTargetAxis() { // unused
        return Utilities.processDeadzone(driver.getTriggerAxis(Hand.kLeft), 0.1) - Utilities.processDeadzone(driver.getTriggerAxis(Hand.kRight), 0.1);
    }

    // Elevator testing
    public static double getElevatorManualA() {
        return Utilities.scale(Utilities.processDeadzone(tester.getY(Hand.kLeft), Tuning.driveDeadzone), 2);
    }

    public static double getElevatorManualB() {
        return Utilities.scale(Utilities.processDeadzone(tester.getY(Hand.kRight), Tuning.driveDeadzone), 2);
    }

    public static boolean getPointDriveLockButton() {
        return OI.lockPointDrive.get();
    }
}
