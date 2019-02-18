package org.team1540.robot2019.commands.selftests;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.TimedCommand;
import org.apache.log4j.Logger;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;

public class ElevatorSelfTest extends TimedCommand {

    public static final Logger logger = Logger.getLogger(ElevatorSelfTest.class);

    public ElevatorSelfTest() {
        super(Tuning.elevatorTestTime);

        requires(Robot.elevator);
    }

    @Override
    protected void initialize() {
        logger.info("Performing elevator self-test");
        Robot.elevator.setRaw(Tuning.elevatorTestThrottle);
    }

    @Override
    protected void end() {
        if (Math.abs(Robot.elevator.getCurrentA() - Robot.elevator.getCurrentB())
            > Tuning.elevatorMaxCurrDiscrepancy) {
            DriverStation.reportWarning("Robot elevator motors seem to have different currents", false);
        } else {
            logger.info("Elevator currents nominal");
        }

        if (Robot.elevator.getVelocity() < Tuning.elevatorTestVelocityThresh) {
            DriverStation.reportWarning("Elevator velocity abnormally low or zero", false);
        } else {
            logger.info("Elevator velocity nominal");
        }

        Robot.elevator.setRaw(0);
    }
}
