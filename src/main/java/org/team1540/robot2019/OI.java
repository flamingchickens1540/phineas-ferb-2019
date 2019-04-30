package org.team1540.robot2019;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.command.Command;
import org.apache.log4j.Logger;
import org.team1540.robot2019.commands.cargo.EjectCargo;
import org.team1540.robot2019.commands.cargo.FloorIntakeCargo;
import org.team1540.robot2019.commands.cargo.ForwardThenEjectCargo;
import org.team1540.robot2019.commands.cargo.LoadingStationIntakeCargo;
import org.team1540.robot2019.commands.elevator.MoveElevatorToPosition;
import org.team1540.robot2019.commands.elevator.MoveElevatorToZero;
import org.team1540.robot2019.commands.wrist.RecoverWrist;
import org.team1540.robot2019.utils.ChickenXboxController;
import org.team1540.robot2019.utils.ChickenXboxController.XboxAxis;
import org.team1540.robot2019.utils.ChickenXboxController.XboxButton;
import org.team1540.rooster.triggers.DPadAxis;
import org.team1540.rooster.util.SimpleCommand;

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

    private static Button cancelIntakeButton = copilot.getButton(XboxAxis.LEFT_TRIG, Tuning.axisButtonThreshold);
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

        cargoIntakeLoadingStationButton.whenPressed(new LoadingStationIntakeCargo());

        // Wrist
        wristRecoverButton.whileHeld(new RecoverWrist());

        // Eject cargo
        cargoEjectButton.whenPressed(new EjectCargo());

        cancelIntakeButton.whenPressed(new SimpleCommand("cancel", Robot.hatch::retract, Robot.cargoMech, Robot.wrist));
        cancelIntakeButton.whenPressed(new MoveElevatorToZero());


        // ---------------------------------------- Driver ----------------------------------------

        // ---------------------------------------- Tester ----------------------------------------

        double end = RobotController.getFPGATime() / 1000.0;
        logger.info("Initialized operator interface in " + (end - start) + " ms");
    }

}
