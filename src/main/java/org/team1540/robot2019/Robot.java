package org.team1540.robot2019;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.apache.log4j.Level;
import org.apache.log4j.Logger;
import org.team1540.robot2019.subsystems.Climber;
import org.team1540.robot2019.subsystems.Drivetrain;
import org.team1540.robot2019.subsystems.Elevator;
import org.team1540.robot2019.subsystems.HatchMech;
import org.team1540.robot2019.subsystems.Intake;
import org.team1540.robot2019.subsystems.Wrist;
import org.team1540.rooster.preferencemanager.PreferenceManager;

public class Robot extends TimedRobot {

  private static final Logger logger = Logger.getLogger(Robot.class);

  public static Drivetrain drivetrain;
  public static Elevator elevator;
  public static Wrist wrist;
  public static Intake intake;
  public static HatchMech hatchMech;
  public static Climber climber;

  @Override
  public void robotInit() {
    // logging configuration
    Logger.getRootLogger().setLevel(Level.ALL);

    logger.info("Initializing...");
    double start = RobotController.getFPGATime() / 1000.0; // getFPGATime returns microseconds

    PreferenceManager.getInstance().add(new Tuning());

    Scheduler.getInstance().run();

    // initialize hardware after we run the scheduler once so that the preference manager can update its values
    Hardware.initAll();

    drivetrain = new Drivetrain();
    elevator = new Elevator();
    wrist = new Wrist();
    intake = new Intake();
    hatchMech = new HatchMech();
    climber = new Climber();

    OI.init();

    // TODO: shuffleboard

    double end = RobotController.getFPGATime() / 1000.0; // getFPGATime returns microseconds
    logger.info("Robot ready. Initialization took " + (end - start) + " ms");
  }

  @Override
  public void robotPeriodic() {
    Scheduler.getInstance().run();

    // TODO: put this on shuffleboard properly
    SmartDashboard.putNumber("System Pressure",
        Hardware.pressureSensor.getVoltage() * Tuning.pressureSensorVoltageScaleFactor);

  }

  private Timer brakeTimer = new Timer();

  @Override
  public void disabledInit() {
    logger.debug("Disabling drive brakes in 2 seconds...");
    brakeTimer.reset();
    brakeTimer.start();

    Hardware.checkStickyFaults();
  }

  @Override
  public void disabledPeriodic() {
    if (brakeTimer.hasPeriodPassed(2)) {
      brakeTimer.stop();
      drivetrain.setBrake(false);
      logger.debug("Drive brakes disabled");
    }
  }

  @Override
  public void autonomousInit() {
    drivetrain.setBrake(true);

    Hardware.checkStickyFaults();
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    drivetrain.setBrake(true);

    Hardware.checkStickyFaults();
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
