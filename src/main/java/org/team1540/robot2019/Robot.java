package org.team1540.robot2019;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import org.team1540.robot2019.subsystems.Drivetrain;
import org.team1540.rooster.preferencemanager.PreferenceManager;

public class Robot extends TimedRobot {
  public static final Drivetrain drivetrain = new Drivetrain();

  @Override
  public void robotInit() {
    PreferenceManager.getInstance().add(new Tuning());

    OI.init();


    // TODO: shuffleboard
  }

  @Override
  public void robotPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }
}
