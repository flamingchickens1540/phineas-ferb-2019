package org.team1540.robot2019;

import static org.team1540.robot2019.OI.LB;
import static org.team1540.robot2019.OI.RB;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.io.IOException;
import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.log4j.Level;
import org.apache.log4j.Logger;
import org.team1540.robot2019.datastructures.Odometry;
import org.team1540.robot2019.datastructures.threed.Transform3D;
import org.team1540.robot2019.networking.UDPOdometryGoalSender;
import org.team1540.robot2019.networking.UDPTwistReceiver;
import org.team1540.robot2019.subsystems.Climber;
import org.team1540.robot2019.subsystems.Drivetrain;
import org.team1540.robot2019.subsystems.Elevator;
import org.team1540.robot2019.subsystems.HatchMech;
import org.team1540.robot2019.subsystems.Intake;
import org.team1540.robot2019.subsystems.Wrist;
import org.team1540.robot2019.utils.LimelightInterface;
import org.team1540.robot2019.utils.LimelightLocalization;
import org.team1540.robot2019.utils.NavxWrapper;
import org.team1540.robot2019.utils.StateChangeDetector;
import org.team1540.robot2019.utils.TankDriveOdometryRunnable;
import org.team1540.robot2019.vision.commands.PurePursuitThenPointToVisionTarget;
import org.team1540.robot2019.vision.commands.UDPVelocityTwistDrive;
import org.team1540.rooster.util.SimpleCommand;

public class Robot extends TimedRobot {

    private static final Logger logger = Logger.getLogger(Robot.class);

    public static Drivetrain drivetrain;
    public static Elevator elevator;
    public static Wrist wrist;
    public static Intake cargoMechanism;
    public static HatchMech hatch;
    public static Climber climber;

    public static boolean debugMode = false;

    boolean disableBrakes;

    public static Transform3D odom_to_base_link = Transform3D.IDENTITY;

  public static TankDriveOdometryRunnable wheelOdometry;

  public static UDPOdometryGoalSender udpSender;
  public static UDPTwistReceiver udpReceiver;
  public static LimelightLocalization limelightLocalization;

    public static Transform3D lastOdomToLimelightGoal;
    public static Transform3D lastOdomToVisionTarget;

    public static LimelightInterface limelight;

  public static NavxWrapper navx = new NavxWrapper();

  // TODO: Move these to OI
  static JoystickButton autoAlignButton = new JoystickButton(OI.driver, RB);
  static JoystickButton autoAlignCancelButton = new JoystickButton(OI.driver, LB);

  static Command alignCommand = null;

  @Override
  public void robotInit() {

      // TODO: Clean this up
      SmartDashboard.putNumber("test-goal/position/x", 2);
      SmartDashboard.putNumber("test-goal/position/y", 0);
      SmartDashboard.putNumber("test-goal/orientation/z", 0);

      SmartDashboard.putNumber("pointkp", 0);
      SmartDashboard.putNumber("pointki", 0);
      SmartDashboard.putNumber("pointkd", 0);

    // logging configuration
    Logger.getRootLogger().setLevel(Level.DEBUG);

        logger.info("Initializing...");
        double start = RobotController.getFPGATime() / 1000.0; // getFPGATime returns microseconds

//    PreferenceManager.getInstance().add(new Tuning());
//    Scheduler.getInstance().run();

        // initialize hardware after we run the scheduler once so that the preference manager can update its values
        Hardware.initAll();

        drivetrain = new Drivetrain();
        elevator = new Elevator();
        wrist = new Wrist();
        cargoMechanism = new Intake();
        hatch = new HatchMech();
        climber = new Climber();

        OI.init();

        ShuffleboardDisplay.init();

        // TODO: Clean this up
    SmartDashboard.putBoolean("rumbleEnabled", true);

    wheelOdometry = new TankDriveOdometryRunnable(
        drivetrain::getLeftPositionMeters,
        drivetrain::getRightPositionMeters,
        Robot.navx::getAngleRadians
    );

    udpReceiver = new UDPTwistReceiver(5801, () -> {
      new Notifier(udpReceiver::attemptConnection).startSingle(1);
    });

    udpSender = new UDPOdometryGoalSender("10.15.40.202", 5800, () -> {
      new Notifier(udpSender::attemptConnection).startSingle(1);
    });

    Robot.limelightLocalization = new LimelightLocalization("limelight-a");
      limelight = new LimelightInterface("limelight-a");


    StateChangeDetector limelightStateDetector = new StateChangeDetector(false);

    // TODO: Clean this up
    new Notifier(() -> {
      wheelOdometry.run();
      Robot.odom_to_base_link = wheelOdometry.getOdomToBaseLink();
      udpSender.setOdometry(new Odometry(Robot.odom_to_base_link, drivetrain.getTwist()));
      Robot.odom_to_base_link.toTransform2D().putToNetworkTable("Odometry/Debug/WheelOdometry");
      boolean targetFound = Robot.limelightLocalization.attemptUpdatePose();
      if (targetFound) {
        Robot.limelightLocalization.getBaseLinkToVisionTarget().toTransform2D().putToNetworkTable("LimelightLocalization/Debug/BaseLinkToVisionTarget");
        Transform3D goal = wheelOdometry.getOdomToBaseLink()
            .add(Robot.limelightLocalization.getBaseLinkToVisionTarget())
            .add(new Transform3D(new Vector3D(-0.65, 0, 0), Rotation.IDENTITY));

          Robot.lastOdomToLimelightGoal = goal;
          Robot.lastOdomToVisionTarget = wheelOdometry.getOdomToBaseLink()
              .add(Robot.limelightLocalization.getBaseLinkToVisionTarget());
        goal.toTransform2D().putToNetworkTable("LimelightLocalization/Debug/BaseLinkToGoal");
      }
      // TODO: Clean this up
      if (alignCommand == null || !alignCommand.isRunning()) {
        if (targetFound && DriverStation.getInstance().isEnabled() && SmartDashboard.getBoolean("rumbleEnabled", true)) {
//          OI.driver.setRumble(RumbleType.kLeftRumble, 1);
        } else {
//          OI.driver.setRumble(RumbleType.kLeftRumble, 0);
        }
      }
      try {
        udpSender.sendIt();
      } catch (IOException e) {
        DriverStation.reportWarning("Unable to send Odometry packet!", false);
      }
    }).startPeriodic(0.011);

    // Testing code
    Command testTEB = new SimpleCommand("Test TEB", () -> {
      new UDPVelocityTwistDrive().start();
    });
    SmartDashboard.putData(testTEB);

    // Testing code
    Command resetWheelOdom = new SimpleCommand("Update PID Values", () -> {
      drivetrain.updatePIDValues();
      System.out.println(Tuning.driveVelocityP);
    });
    resetWheelOdom.setRunWhenDisabled(true);
    SmartDashboard.putData(resetWheelOdom);

    autoAlignButton.whenPressed(new SimpleCommand("Start Lineup", () -> {
//      alignCommand = new PurePursuitToVisionTarget(Robot.limelightLocalization, Robot.wheelOdometry);
//        alignCommand = new SimplePointToVisionTarget();
        alignCommand = new PurePursuitThenPointToVisionTarget();
      alignCommand.start();
    }));
    autoAlignCancelButton.whenPressed(new SimpleCommand("Cancel Lineup", () -> {
      if (alignCommand != null) {
        alignCommand.cancel();
      }
    }));

    NetworkTable tebConfigTable = NetworkTableInstance.getDefault().getTable("TEBPlanner/Config");
    tebConfigTable.getEntry("ResetTuningVals").setBoolean(true);

    double end = RobotController.getFPGATime() / 1000.0; // getFPGATime returns microseconds
    logger.info("Robot ready. Initialization took " + (end - start) + " ms");
SmartDashboard.putBoolean("IsHatchPreload", false);
        SmartDashboard.putBoolean("Debug Mode", false);
    }

    @Override
    public void robotPeriodic() {
//    long time = System.currentTimeMillis();
        Scheduler.getInstance().run();
//    System.out.println(time - System.currentTimeMillis());

        debugMode = SmartDashboard.getBoolean("Debug Moe", false);
    }

    private Timer brakeTimer = new Timer();

    @Override
    public void disabledInit() {
        logger.debug("Disabling mechanism brakes in 2 seconds...");
        brakeTimer.reset();
        brakeTimer.start();
        disableBrakes = true;

        Robot.hatch.retract();

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

        if (elevator.getPosition() < 1) {
            elevator.setRaw(0);
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        setMechanismBrakes(true);

        Robot.climber.cylinderUp();

        Hardware.checkStickyFaults();

        if (DriverStation.getInstance().isFMSAttached()) {
            Shuffleboard.startRecording();
        }

        Shuffleboard.addEventMarker("Teleop Start", EventImportance.kNormal);

        if (elevator.getPosition() < 1 && elevator.getCurrentCommand() == null) {
            elevator.setRaw(0);
        }

        if (SmartDashboard.getBoolean("IsHatchPreload", false)) {
            Robot.hatch.extend();
        }
    }

    @Override
    public void teleopPeriodic() {
        if ((Robot.elevator.getPosition() > Tuning.elevatorTolerance)
            && (Robot.climber.getCurrentCommand() == null)) {
            if (Hardware.compressor.getClosedLoopControl()) {
                logger.debug("Stopping compressor because elevator is up");
                Hardware.compressor.stop();
            }
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
