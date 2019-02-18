package org.team1540.robot2019.tuners;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import org.team1540.robot2019.Hardware;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.subsystems.Cargo;
import org.team1540.rooster.preferencemanager.PreferenceManager;

public class IntakeTuningRobot extends TimedRobot {

    private Joystick joystick = new Joystick(0);
    private Cargo cargo;

    @Override
    public void robotInit() {
        PreferenceManager.getInstance().add(new Tuning());

        Scheduler.getInstance().run();

        Hardware.initIntake();

        cargo = new Cargo();
    }

    @Override
    public void teleopPeriodic() {
        if (joystick.getRawButton(1)) {
            cargo.startIntaking();
        } else if (joystick.getRawButton(2)) {
            cargo.startEjecting();
        } else {
            cargo.stop();
        }
    }

    @Override
    public void robotPeriodic() {
        Scheduler.getInstance().run();
    }
}
