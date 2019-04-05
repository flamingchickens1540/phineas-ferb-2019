package org.team1540.robot2019.commands.selftests;

import edu.wpi.first.wpilibj.command.TimedCommand;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;

public class ElevatorSelfTestEachMotor extends TimedCommand {

    private boolean motor1;

    public ElevatorSelfTestEachMotor(boolean motor1) {
        super(Tuning.elevatorTestTime);
        this.motor1 = motor1;
        requires(Robot.elevator);
    }


    @Override
    protected void initialize() {
        Robot.elevator.setRaw(motor1 ? Tuning.elevatorTestThrottle : 0,
            motor1 ? 0 : Tuning.elevatorTestThrottle);
    }

    @Override
    protected void end() {
        Robot.elevator.setRaw(0, 0);
        Robot.elevator.joinFollowers();
    }
}
