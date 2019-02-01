package org.team1540.robot2019;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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
import org.team1540.rooster.util.SimpleCommand;

public class Robot extends TimedRobot {

  private static final Logger logger = Logger.getLogger(Robot.class);

  public static Drivetrain drivetrain;
  public static Elevator elevator;
  public static Wrist wrist;
  public static Intake intake;
  public static HatchMech hatchMech;
  public static Climber climber;

  boolean disableBrakes;

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

    Shuffleboard.getTab("Phineas")
        .add(new SimpleCommand("Reset Preferences", Preferences.getInstance()::removeAll));

    // TODO: shuffleboard

    double end = RobotController.getFPGATime() / 1000.0; // getFPGATime returns microseconds
    logger.info("Robot ready. Initialization took " + (end - start) + " ms");
  }

  @Override
  public void robotPeriodic() {
    Scheduler.getInstance().run();

    // TODO: put this on shuffleboard properly
    SmartDashboard.putNumber("System Pressure", 50 * (Hardware.pressureSensor.getVoltage() - 0.5));
  }

  private Timer brakeTimer = new Timer();

  @Override
  public void disabledInit() {
    logger.debug("Disabling drive brakes in 2 seconds...");
    brakeTimer.reset();
    brakeTimer.start();
    disableBrakes = true;

    wrist.handleDisable();

    if (DriverStation.getInstance().isFMSAttached()) {
      logger.debug("FMS is attached, auto-stopping recording");
      Shuffleboard.stopRecording();
    }

    Shuffleboard.addEventMarker("Robot Disable", EventImportance.kNormal);

    Hardware.checkStickyFaults();
  }

  @Override
  public void disabledPeriodic() {
    if (brakeTimer.hasPeriodPassed(2) && disableBrakes) {
      brakeTimer.stop();
      drivetrain.setBrake(false);
      logger.debug("Drive brakes disabled");
      disableBrakes = false;

      Shuffleboard.addEventMarker("Drive brakes disabled", EventImportance.kTrivial);
    }
  }

  @Override
  public void autonomousInit() {
    drivetrain.setBrake(true);

    Hardware.checkStickyFaults();

    if (DriverStation.getInstance().isFMSAttached()) {
      logger.debug("FMS is attached, auto-starting recording");
      Shuffleboard.setRecordingFileNameFormat(
          DriverStation.getInstance().getEventName() + "-" + DriverStation.getInstance()
              .getMatchType() + "-" + DriverStation.getInstance().getMatchNumber()
              + "-${date}-${time}");

      Shuffleboard.startRecording();
    }

    Shuffleboard.addEventMarker("Autonomous Start", EventImportance.kNormal);
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    drivetrain.setBrake(true);

    Hardware.checkStickyFaults();

    if (DriverStation.getInstance().isFMSAttached()) {
      Shuffleboard.startRecording();
    }

    Shuffleboard.addEventMarker("Teleop Start", EventImportance.kNormal);
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
