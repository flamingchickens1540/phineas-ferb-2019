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
import org.team1540.robot2019.commands.elevator.MoveElevatorToPosition;
import org.team1540.robot2019.commands.elevator.MoveElevatorToZero;
import org.team1540.robot2019.commands.hatch.GrabHatchThenBack;
import org.team1540.robot2019.commands.hatch.PrepHatchFloorGrab;
import org.team1540.robot2019.commands.hatch.StowHatchMech;
import org.team1540.robot2019.commands.hatch.TestGrabHatch;
import org.team1540.robot2019.commands.hatch.TestPlaceHatch;
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
    private static XboxController copilot = new XboxController(1);

    // copilot buttons

    private static Button elevatorMidRocketButton = new StrictDPadButton(copilot, 0, DPadAxis.UP);
    private static Button elevatorCargoShipButton = new StrictDPadButton(copilot, 0, DPadAxis.LEFT);
    private static Button elevatorDownButton = new StrictDPadButton(copilot, 0, DPadAxis.DOWN);
    private static Button intakeLoadingStationButton = new StrictDPadButton(copilot, 0, DPadAxis.RIGHT);

    private static JoystickButton autoIntakeButton = new JoystickButton(copilot, A);
    private static Button cancelIntakeButton = new AxisButton(copilot, Tuning.axisButtonThreshold, LEFT_Y);
    private static JoystickButton ejectButton = new JoystickButton(copilot, B);

    private static JoystickButton prepGetHatchButton = new JoystickButton(copilot, X);
    private static JoystickButton prepGetHatchFloorButton = new JoystickButton(copilot, START);
    private static Button grabHatchButton = new AxisButton(copilot, Tuning.axisButtonThreshold, RIGHT_TRIG);
    private static JoystickButton placeHatchButton = new JoystickButton(copilot, Y);
    private static Button stowHatchButton = new AxisButton(copilot, Tuning.axisButtonThreshold, LEFT_TRIG);

    private static Button climbingSafety = new AxisButton(copilot, Tuning.axisButtonThreshold, LEFT_TRIG);
    private static JoystickButton climbLevel3Button = new JoystickButton(copilot, RB); // + safety
    private static JoystickButton climbLevel2Button = new JoystickButton(copilot, LB); // + safety
    private static JoystickButton climberCylinderUp = new JoystickButton(copilot, BACK);
    private static Button cancelClimbButton = new AxisButton(copilot, -Tuning.axisButtonThreshold, LEFT_Y);

    // driver buttons

    public static JoystickButton quickTurnButton = new JoystickButton(driver, LB);
    //    public static JoystickButton autoAlignButtonAlt = new JoystickButton(driver, RB);
    public static JoystickButton autoAlignButton = new JoystickButton(driver, RB);
    public static MultiAxisButton autoAlignCancelAxisButton = new MultiAxisButton(driver, 0.7, new int[]{LEFT_TRIG, RIGHT_TRIG, RIGHT_X, RIGHT_Y});
    public static JoystickButton autoAlignManualCancelButton = new JoystickButton(driver, X);

    public static AxisButton autoAlignPointButton = new AxisButton(driver, Tuning.axisButtonThreshold, LEFT_TRIG);
//    public static AxisButton testPlaceHatchButton = new AxisButton(driver, Tuning.axisButtonThreshold, RIGHT_TRIG);
    public static JoystickButton testGrabHatchButton = new JoystickButton(driver, A);
    public static JoystickButton testPlaceHatchButton = new JoystickButton(driver, B);
    private static Button testElevatorMidRocketButton = new StrictDPadButton(driver, 0, DPadAxis.UP);

    public static JoystickButton resetPointOffset = new JoystickButton(driver, Y);
    public static PercentManualLineupSequence alignCommand;

    public static JoystickButton arcadeToggle = new JoystickButton(driver, BACK);

//    private static Button autoAlignButtonAlt = new AxisButton(driver, Tuning.axisButtonThreshold, RIGHT_TRIG);

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
        intakeLoadingStationButton.whenPressed(new SimpleConditionalCommand(Robot.hatch::isReleased, loadingIntakeCommand));
        cancelIntakeButton.cancelWhenPressed(loadingIntakeCommand);
        cancelIntakeButton.whenPressed(new MoveElevatorToZero());

        Command intakeCommand = new FloorIntake();
        autoIntakeButton.whenPressed(new SimpleConditionalCommand(Robot.hatch::isReleased, intakeCommand));
        cancelIntakeButton.cancelWhenPressed(intakeCommand);
        ForwardThenEject command = new ForwardThenEject();
        ejectButton.whenPressed(command);
        ejectButton.whenReleased(new BackThenDown());
        ejectButton.whenReleased(new SimpleCommand("", command::cancel));

        prepGetHatchButton.whenPressed(new TestGrabHatch());
        placeHatchButton.whenPressed(new TestPlaceHatch());

        prepGetHatchFloorButton.whenPressed(new PrepHatchFloorGrab());
        grabHatchButton.whenPressed(new GrabHatchThenBack());
        stowHatchButton.whenPressed(new StowHatchMech());

        Command climbCommand3 = new ClimbLevelThree();
        Command climbCommand2 = new ClimbLevelTwo();
        climbLevel3Button.whenPressed(new SimpleConditionalCommand(climbingSafety::get, climbCommand3));
        climbLevel2Button.whenPressed(new SimpleConditionalCommand(climbingSafety::get, climbCommand2));
        climberCylinderUp.whenPressed(new SimpleCommand("Raise Cylinder", Robot.climber::raiseCylinder, Robot.climber));
        cancelClimbButton.cancelWhenPressed(climbCommand3);
        cancelClimbButton.cancelWhenPressed(climbCommand2);

        alignCommand = new PercentManualLineupSequence();
        autoAlignButton.whenPressed(alignCommand);
//        autoAlignButtonAlt.whenPressed(alignCommand);

//        autoAlignButton.whenReleased(new SimpleCommand("Cancel Auto-lineup", alignCommand::cancel));
//        autoAlignButtonAlt.whenReleased(new SimpleCommand("Cancel Auto-lineup", alignCommand::cancel));
        autoAlignCancelAxisButton.cancelWhenPressed(alignCommand);
        elevatorMidRocketButton.cancelWhenPressed(alignCommand);
        elevatorCargoShipButton.cancelWhenPressed(alignCommand);
        intakeLoadingStationButton.cancelWhenPressed(alignCommand);
        autoAlignManualCancelButton.cancelWhenPressed(alignCommand);

        autoAlignPointButton.whenPressed(alignCommand);

//        SimplePointToAngle quickTurnCommand = new SimplePointToAngle(Math.PI - Math.toRadians(2));
//        quickTurnButton.whenPressed(quickTurnCommand);
//        autoAlignCancelAxisButton.cancelWhenPressed(quickTurnCommand);

        testGrabHatchButton.whenPressed(new TestGrabHatch());
        testPlaceHatchButton.whenPressed(new TestPlaceHatch());
        testElevatorMidRocketButton.whenPressed(new MoveElevatorToPosition(Tuning.elevatorUpPosition));
        testElevatorMidRocketButton.cancelWhenPressed(alignCommand);

        Command arcadeCommand = new SimpleLoopCommand("Drive",
            new AdvancedArcadeJoystickInput(true, OI::getDriveThrottle, OI::getDriveSoftTurn,
                OI::getDriveHardTurn)
                .then(new FeedForwardToVelocityProcessor(Tuning.driveMaxVel))
                .then(new FeedForwardProcessor(Tuning.driveKV, Tuning.driveVIntercept, 0))
                .then(Robot.drivetrain.getPipelineOutput(false)), Robot.drivetrain);
        arcadeToggle.whenPressed(new SimpleCommand("Toggle Auto Lineup", () -> {
            if (arcadeCommand.isRunning()) {
                arcadeCommand.cancel();
            } else {
                arcadeCommand.start();
            }
        }));


        resetPointOffset.whenPressed(new SimpleCommand("Reset Point Offset", () -> {
            PointDrive.setInitAngleOffset(Hardware.navx.getYawRadians());
        }));

        double end = RobotController.getFPGATime() / 1000.0;
        logger.info("Initialized operator interface in " + (end - start) + " ms");
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

    public static double getClimbAxis() {
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
        double x = -driver.getY(Hand.kRight);
        double y = -driver.getX(Hand.kRight);
        return Math.atan2(y, x);
    }

    public static double getPointDriveMagnitude() {
        double x = driver.getX(Hand.kRight);
        double y = driver.getY(Hand.kRight);
        return Utilities.processDeadzone(new Vector2D(x, y).distance(Vector2D.ZERO), Tuning.driveDeadzone);
    }
}
