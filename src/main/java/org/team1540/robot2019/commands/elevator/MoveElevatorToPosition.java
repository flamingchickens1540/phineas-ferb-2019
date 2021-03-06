package org.team1540.robot2019.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import org.apache.log4j.Logger;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;

public class MoveElevatorToPosition extends Command {

    private static final Logger logger = Logger.getLogger(MoveElevatorToPosition.class);

    private double position;

    public MoveElevatorToPosition(double pos) {
        this.position = pos;
        requires(Robot.elevator);
    }

    @Override
    protected void initialize() {
        logger.debug("Moving elevator to position " + position);
        Robot.elevator.setPosition(position);
    }

    @Override
    protected void end() {
        logger.debug("Elevator movement to " + position + " complete");
    }

    @Override
    protected void interrupted() {
        logger.debug("Elevator movement to " + position + " interrupted");
    }

    @Override
    protected boolean isFinished() {
        return Math.abs(Robot.elevator.getPosition() - position) < Tuning.elevatorTolerance
            && Math.abs(Robot.elevator.getVelocity()) < Tuning.elevatorVelocityTolerance;
    }
}
