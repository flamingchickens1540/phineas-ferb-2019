package org.team1540.robot2019;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Scheduler;
import org.team1540.robot2019.subsystems.*;
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
    System.out.println("Initializing...");
    double start = RobotController.getFPGATime() / 1000.0; // getFPGATime returns microseconds

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

    double end = RobotController.getFPGATime() / 1000.0; // getFPGATime returns microseconds
    System.out.println("Robot ready. Initialization took " + (end - start) + " ms");
  }

  @Override
  public void robotPeriodic() {
    Scheduler.getInstance().run();
  }

  private Timer brakeTimer = new Timer();

  @Override
  public void disabledInit() {
    System.out.println("Disabling drive brakes in 2 seconds...");
    brakeTimer.reset();
    brakeTimer.start();
  }

  @Override
  public void disabledPeriodic() {
    if (brakeTimer.hasPeriodPassed(2)) {
      brakeTimer.stop();
      drivetrain.setBrake(false);
      System.out.println("Drive brakes disabled");
    }
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
