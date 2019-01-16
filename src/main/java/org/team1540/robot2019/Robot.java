package org.team1540.robot2019;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import org.team1540.robot2019.subsystems.Arm;
import org.team1540.robot2019.subsystems.Drivetrain;
import org.team1540.robot2019.subsystems.Elevator;
import org.team1540.robot2019.subsystems.Intake;
import org.team1540.rooster.preferencemanager.PreferenceManager;

public class Robot extends TimedRobot {

  public static Drivetrain drivetrain;
  public static Elevator elevator;
  public static Arm arm;
  public static Intake intake;

  @Override
  public void robotInit() {
    PreferenceManager.getInstance().add(new Tuning());

    Scheduler.getInstance().run();

    // initialize hardware after we run the scheduler once so that the preference manager can update its values
    Hardware.initAll();

    drivetrain = new Drivetrain();
    elevator = new Elevator();
    arm = new Arm();
    intake = new Intake();

    OI.init();

    // TODO: shuffleboard
  }

  @Override
  public void robotPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    drivetrain.setBrake(false);
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    drivetrain.setBrake(true);
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    drivetrain.setBrake(true);
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
