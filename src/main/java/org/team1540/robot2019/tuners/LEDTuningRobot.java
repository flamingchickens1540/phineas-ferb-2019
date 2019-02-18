package org.team1540.robot2019.tuners;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import org.team1540.robot2019.Hardware;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.subsystems.Intake;
import org.team1540.rooster.preferencemanager.PreferenceManager;

public class LEDTuningRobot extends TimedRobot {

    private Joystick joystick = new Joystick(1);
    private Solenoid s1 = new Solenoid(6);
    private Solenoid s2 = new Solenoid(7);


    @Override
    public void robotInit() {
        PreferenceManager.getInstance().add(new Tuning());

        Scheduler.getInstance().run();
    }

    @Override
    public void teleopPeriodic() {
        s1.set(joystick.getRawButton(1));
        s2.set(joystick.getRawButton(2));
    }

    @Override
    public void robotPeriodic() {
        Scheduler.getInstance().run();
    }
}
