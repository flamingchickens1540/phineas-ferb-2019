package org.team1540.robot2019;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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

    ShuffleboardDisplay.init();

    double end = RobotController.getFPGATime() / 1000.0; // getFPGATime returns microseconds
    logger.info("Robot ready. Initialization took " + (end - start) + " ms");
  }

  @Override
  public void robotPeriodic() {
    Scheduler.getInstance().run();
  }

  private Timer brakeTimer = new Timer();

  @Override
  public void disabledInit() {
    logger.debug("Disabling mechanism brakes in 2 seconds...");
    brakeTimer.reset();
    brakeTimer.start();
    disableBrakes = true;

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
      setMechanismBrakes(false);

      logger.debug("Mechanism brakes disabled");

      disableBrakes = false;

      Shuffleboard.addEventMarker("Mechanism brakes disabled", EventImportance.kTrivial);
    }
  }

  @Override
  public void autonomousInit() {
    setMechanismBrakes(true);

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
    setMechanismBrakes(true);

    Hardware.checkStickyFaults();

    if (DriverStation.getInstance().isFMSAttached()) {
      Shuffleboard.startRecording();
    }

    Shuffleboard.addEventMarker("Teleop Start", EventImportance.kNormal);
  }

  @Override
  public void teleopPeriodic() {
    if (Hardware.compressor.getClosedLoopControl()
        && (Robot.elevator.getPosition() > Tuning.elevatorTolerance)
        && (Robot.climber.getCurrentCommand() == null)) {
      logger.debug("Stopping compressor because elevator is up");
      Hardware.compressor.stop();
    } else if (!Hardware.compressor.getClosedLoopControl()) {
      logger.debug("Restarting compressor");
      Hardware.compressor.start();
    }
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  private void setMechanismBrakes(boolean b) {
    drivetrain.setBrake(b);
    elevator.setBrake(b);
    climber.setArmBrake(b);
    wrist.setBrake(b);
  }
}
