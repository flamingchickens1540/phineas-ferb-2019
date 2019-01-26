package org.team1540.robot2019;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import org.team1540.robot2019.subsystems.*;
import org.team1540.rooster.preferencemanager.PreferenceManager;

public class DriveTestRobot extends TimedRobot {

  private static Drivetrain drivetrain;
  private Joystick joystick = new Joystick(0);

  @Override
  public void robotInit() {
    PreferenceManager.getInstance().add(new Tuning());
    Hardware.initDrive();
    OI.initJoysticks();

    drivetrain = new Drivetrain();
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void robotPeriodic() {
    Scheduler.getInstance().run();
  }
}
