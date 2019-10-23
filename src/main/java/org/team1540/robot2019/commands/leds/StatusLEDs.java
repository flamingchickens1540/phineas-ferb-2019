package org.team1540.robot2019.commands.leds;

import edu.wpi.first.wpilibj.command.Command;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;

public class StatusLEDs extends Command {
    public StatusLEDs() {
        requires(Robot.leds);
    }

    @Override
    protected void initialize() {
        requires(Robot.leds);
        Robot.leds.set(Tuning.idleLEDs);
    }

    @Override
    protected void execute() {
        if (Robot.cargoMech.getCurrentCommandName() == "FloorIntakeCargo") {
            Robot.leds.set(Tuning.floorIntakeCargoLEDs);
        }   else if (Robot.cargoMech.hasBall()) {
            Robot.leds.set(Tuning.hasBallLEDs);
        }   else if (Math.abs(Robot.elevator.getPosition() - Tuning.elevatorCargoShipPosition) <= 1) {
            Robot.leds.set(Tuning.elevatorCargoShipPositionLEDs);
        }   else if (Math.abs(Robot.elevator.getPosition() - Tuning.elevatorUpPosition) <= 1) {
            Robot.leds.set(Tuning.elevatorTopLEDs);
        } else {
            Robot.leds.set(Tuning.idleLEDs);
        }
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
