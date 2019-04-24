package org.team1540.robot2019.tuners;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import org.apache.log4j.Level;
import org.apache.log4j.Logger;
import org.team1540.robot2019.Hardware;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.subsystems.Elevator;
import org.team1540.rooster.preferencemanager.PreferenceManager;

public class ElevatorTestingRobot extends TimedRobot {

    private Joystick joystick = new Joystick(0);

    @Override
    public void robotInit() {
        Logger.getRootLogger().setLevel(Level.DEBUG);
        PreferenceManager.getInstance().add(new Tuning());
        Scheduler.getInstance().run();

        Hardware.unfollowInitElevator();

        Robot.elevator = new Elevator();
    }

    @Override
    public void teleopPeriodic() {
//        Hardware.elevatorA.set(OI.getElevatorManualA());
//        Hardware.elevatorB.set(OI.getElevatorManualB());
    }

    public void teleopInit() {
        Robot.elevator.disableMotors();
    }

    @Override
    public void robotPeriodic() {
        Scheduler.getInstance().run();
    }
}
