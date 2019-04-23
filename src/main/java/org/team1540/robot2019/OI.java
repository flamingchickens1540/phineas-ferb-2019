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
import org.team1540.robot2019.commands.cargo.*;
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
    private static ChickenXboxController copilot = new ChickenXboxController(1);

    // ---------------------------------------- Copilot ----------------------------------------

    // Elevator
    private static Button elevatorFullUpButton = copilot.getButton(DPadAxis.UP);
    private static Button elevatorCargoShipButton = copilot.getButton(DPadAxis.LEFT);
    private static Button elevatorDownButton = copilot.getButton(DPadAxis.DOWN);

    // Cargo
    private static Button cargoFloorIntakeButton = copilot.getButton(XboxButton.A);
    private static Button cargoIntakeLoadingStationButton = copilot.getButton(XboxButton.X);

    private static Button cargoEjectButton = copilot.getButton(XboxButton.B);

    private static Button wristRecoverButton = copilot.getButton(XboxAxis.LEFT_Y, Tuning.axisButtonThreshold);

    // ---------------------------------------- Driver ----------------------------------------

    // ---------------------------------------- Tester ----------------------------------------


    /**
     * Since we want to initialize stuff once the robot actually boots up (not as static initializers), we instantiate stuff here to get more informative error traces and less general weirdness.
     */
    static void init() {
        logger.info("Initializing operator interface...");
        double start = RobotController.getFPGATime() / 1000.0; // getFPGATime returns microseconds

        // ---------------------------------------- Copilot ----------------------------------------

        // Elevator
        MoveElevatorToPosition moveElevatorUp = new MoveElevatorToPosition(Tuning.elevatorUpPosition);
        elevatorFullUpButton.whenPressed(moveElevatorUp);
        MoveElevatorToPosition moveElevatorToCargoShip = new MoveElevatorToPosition(Tuning.elevatorCargoShipPosition);
        elevatorCargoShipButton.whenPressed(moveElevatorToCargoShip);
        MoveElevatorToZero moveElevatorToZero = new MoveElevatorToZero();
        elevatorDownButton.whenPressed(moveElevatorToZero);

        // Cargo
        Command cargoFloorIntake = new FloorIntakeCargo();
        cargoFloorIntakeButton.whenPressed(cargoFloorIntake);

        cargoIntakeLoadingStationButton.whenPressed(new IntakeCargo());

        // Wrist
        wristRecoverButton.whileHeld(new RecoverWrist());

        // Eject cargo
        cargoEjectButton.whenPressed(new EjectCargo());


        // ---------------------------------------- Driver ----------------------------------------

        // ---------------------------------------- Tester ----------------------------------------

        double end = RobotController.getFPGATime() / 1000.0;
        logger.info("Initialized operator interface in " + (end - start) + " ms");
    }

}
