package org.team1540.robot2019.tuners;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import org.team1540.robot2019.Hardware;
import org.team1540.robot2019.OI;
import org.team1540.robot2019.subsystems.Drivetrain;

public class DriveTuningRobot extends TimedRobot {

    private static Drivetrain drivetrain;
    private Joystick joystick = new Joystick(0);

    @Override
    public void robotInit() {
        Hardware.initDrive();
        OI.initJoysticks();

        drivetrain = new Drivetrain();
    }

    @Override
    public void robotPeriodic() {
        Scheduler.getInstance().run();
    }
}
