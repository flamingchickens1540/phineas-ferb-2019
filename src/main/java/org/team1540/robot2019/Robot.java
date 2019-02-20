package org.team1540.robot2019;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.apache.log4j.Level;
import org.apache.log4j.Logger;
import org.team1540.robot2019.datastructures.Odometry;
import org.team1540.robot2019.datastructures.TransformManager;
import org.team1540.robot2019.datastructures.threed.Transform3D;
import org.team1540.robot2019.odometry.tankdrive.TankDriveOdometryRunnable;
import org.team1540.robot2019.subsystems.Climber;
import org.team1540.robot2019.subsystems.Drivetrain;
import org.team1540.robot2019.subsystems.Elevator;
import org.team1540.robot2019.subsystems.HatchMech;
import org.team1540.robot2019.subsystems.Intake;
import org.team1540.robot2019.subsystems.LEDs;
import org.team1540.robot2019.subsystems.Wrist;
import org.team1540.robot2019.utils.LastValidTransformTracker;
import org.team1540.robot2019.vision.deepspace.DeepSpaceVisionTargetLocalization;
import org.team1540.robot2019.wrappers.Limelight;
import org.team1540.robot2019.wrappers.TEBPlanner;

public class Robot extends TimedRobot {

    private static final Logger logger = Logger.getLogger(Robot.class);

    public static Drivetrain drivetrain;
    public static Elevator elevator;
    public static Wrist wrist;
    public static Intake cargoMechanism;
    public static HatchMech hatch;
    public static Climber climber;
    public static LEDs leds;

    public static boolean debugMode = false;

    boolean disableBrakes;

    public static TankDriveOdometryRunnable odometry;
    public static DeepSpaceVisionTargetLocalization deepSpaceVisionTargetLocalization;
    public static Limelight limelight;
    public static TEBPlanner tebPlanner;
    public static LastValidTransformTracker lastOdomToVisionTargetTracker;
    public static TransformManager tf;

    @Override
    public void robotInit() {
        // logging configuration
        Logger.getRootLogger().setLevel(Level.DEBUG);

        logger.info("Initializing...");
        double start = RobotController.getFPGATime() / 1000.0; // getFPGATime returns microseconds

//    PreferenceManager.getInstance().add(new Tuning());
//    Scheduler.getInstance().run();

        // initialize hardware after we run the scheduler once so that the preference manager can update its values
        Hardware.initAll();

        // SUBSYSTEMS
        drivetrain = new Drivetrain();
        elevator = new Elevator();
        wrist = new Wrist();
        cargoMechanism = new Intake();
        hatch = new HatchMech();
        climber = new Climber();

        // TRANSFORMS
        tf = new TransformManager();

        // ODOMETRY
        odometry = new TankDriveOdometryRunnable(
            drivetrain::getLeftPositionMeters,
            drivetrain::getRightPositionMeters,
            Hardware.navx::getAngleRadians,
            "odom",
            "base_link",
            tf,
            0.011
        );

        // VISION
        limelight = new Limelight("limelight-a", new Transform3D(0.086, 0.099, 1.12, Tuning.CAM_ROLL, Tuning.CAM_PITCH, 0));
        lastOdomToVisionTargetTracker = new LastValidTransformTracker(() -> tf.getTransform("odom", "base_link"));
        deepSpaceVisionTargetLocalization = new DeepSpaceVisionTargetLocalization(limelight, 0.71, 0.05,
            lastOdomToVisionTargetTracker); // Doesn't have to be very frequent if things that use it also call update

        // PLANNING
        tebPlanner = new TEBPlanner(() -> new Odometry(tf.getTransform("odom", "base_link"), drivetrain.getTwist()), 5801, 5800, "10.15.40.202", 0.01);

        OI.init();

        ShuffleboardDisplay.init();

        double end = RobotController.getFPGATime() / 1000.0; // getFPGATime returns microseconds
        logger.info("Robot ready. Initialization took " + (end - start) + " ms");

        SmartDashboard.putBoolean("IsHatchPreload", false);
        SmartDashboard.putBoolean("Debug Mode", false);
    }

    @Override
    public void robotPeriodic() {
        Scheduler.getInstance().run();

        debugMode = SmartDashboard.getBoolean("Debug Mode", false);
        tf.getTransform("odom", "base_link").toTransform2D().putToNetworkTable("Odometry/Debug");
        lastOdomToVisionTargetTracker.getOdomToVisionTarget().toTransform2D().putToNetworkTable("DeepSpaceVisionTargetLocalization/Debug/OdomToVisionTarget");
        deepSpaceVisionTargetLocalization.getLastBaseLinkToVisionTarget().toTransform2D().putToNetworkTable("DeepSpaceVisionTargetLocalization/Debug/BaseLinkToVisionTarget");
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
        if (!SmartDashboard.getBoolean("IsHatchPreload", true)) {
            Robot.hatch.release();
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
