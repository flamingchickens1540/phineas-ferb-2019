package org.team1540.robot2019;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
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
import org.team1540.robot2019.commands.auto.UDPVelocityTwistDrive;
import org.team1540.robot2019.datastructures.Odometry;
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
import org.team1540.rooster.util.SimpleCommand;

public class Robot extends TimedRobot {

    private static final Logger logger = Logger.getLogger(Robot.class);

    public static Drivetrain drivetrain;
    public static Elevator elevator;
    public static Wrist wrist;
    public static Intake intake;
    public static HatchMech hatch;
    public static Climber climber;
    public static LEDs leds;

    public static boolean debugMode = false;

    boolean disableBrakes;
    private Timer brakeTimer = new Timer();

    public static TankDriveOdometryRunnable odometry;
    public static DeepSpaceVisionTargetLocalization deepSpaceVisionTargetLocalization;
    public static Limelight limelight;
    public static TEBPlanner tebPlanner;
    public static LastValidTransformTracker lastOdomToVisionTargetTracker;

    @Override
    public void robotInit() {
        // logging configuration
        Logger.getRootLogger().setLevel(Level.DEBUG);

        logger.info("Initializing...");
        double start = RobotController.getFPGATime() / 1000.0; // getFPGATime returns microseconds

        Hardware.initAll();

        drivetrain = new Drivetrain();
        elevator = new Elevator();
        wrist = new Wrist();
        intake = new Intake();
        hatch = new HatchMech();
        climber = new Climber();
        leds = new LEDs();

        odometry = new TankDriveOdometryRunnable(
            drivetrain::getLeftPositionMeters,
            drivetrain::getRightPositionMeters,
            Hardware.navx::getAngleRadians,
            0.011
        );

        limelight = new Limelight("limelight-a",
            new Transform3D(RobotMap.CAM_X, RobotMap.CAM_Y, RobotMap.CAM_Z, RobotMap.CAM_ROLL,
                RobotMap.CAM_PITCH, RobotMap.CAM_YAW));
        lastOdomToVisionTargetTracker = new LastValidTransformTracker(odometry::getOdomToBaseLink);
        deepSpaceVisionTargetLocalization = new DeepSpaceVisionTargetLocalization(limelight,
            RobotMap.HATCH_TARGET_HEIGHT, 0.05,
            lastOdomToVisionTargetTracker); // Doesn't have to be very frequent if things that use it also call update

        tebPlanner = new TEBPlanner(() -> new Odometry(odometry.getOdomToBaseLink(), drivetrain.getTwist()), 5801, 5800,
            "10.15.40.76", 0.01);

        OI.init();

        ShuffleboardDisplay.init();

        // TODO: use shuffleboard properly
        SmartDashboard.putBoolean("IsHatchPreload", false);
        SmartDashboard.putBoolean("Debug Mode", false);

        SmartDashboard.putBoolean("EnableCompressor", true);

        Robot.limelight.prepForDriverCam();

        SmartDashboard.setDefaultBoolean("EnableBackupCam", false);
        if (SmartDashboard.getBoolean("EnableBackupCam", false)) {
            UsbCamera cam = CameraServer.getInstance().startAutomaticCapture("backup cam", 0);
            cam.setResolution(128, 73);
            cam.setFPS(30);
        }

        double end = RobotController.getFPGATime() / 1000.0; // getFPGATime returns microseconds
        logger.info("Robot ready. Initialization took " + (end - start) + " ms");

        SimpleCommand testTeb = new SimpleCommand("Test TEB", () -> {
            new UDPVelocityTwistDrive(false).start();
        });
        SmartDashboard.putData(testTeb);
    }

    @Override
    public void robotPeriodic() {
        Scheduler.getInstance().run();

        debugMode = SmartDashboard.getBoolean("Debug Mode", false);
        odometry.getOdomToBaseLink().toTransform2D().putToNetworkTable("Odometry/Debug");
        if (lastOdomToVisionTargetTracker.getOdomToVisionTarget() != null) {
            lastOdomToVisionTargetTracker.getOdomToVisionTarget().toTransform2D()
                .putToNetworkTable("DeepSpaceVisionTargetLocalization/Debug/OdomToVisionTarget");

        }
        Transform3D lastBaseLinkToVisionTarget = deepSpaceVisionTargetLocalization.getLastBaseLinkToVisionTarget();
        if (lastBaseLinkToVisionTarget != null) {
            lastBaseLinkToVisionTarget.toTransform2D()
                .putToNetworkTable("DeepSpaceVisionTargetLocalization/Debug/BaseLinkToVisionTarget");
        }

        SmartDashboard.putNumber("DrivetrainLeftPos", drivetrain.getLeftPositionTicks());
        SmartDashboard.putNumber("DrivetrainRightPos", drivetrain.getRightPositionTicks());
    }

    @Override
    public void disabledInit() {
        logger.debug("Disabling mechanism brakes in 2 seconds...");
        brakeTimer.reset();
        brakeTimer.start();
        disableBrakes = true;

        Robot.hatch.retract();
        Robot.hatch.grab(); // otherwise we might flicker grab-release on enable

        if (DriverStation.getInstance().isFMSAttached()) {
            logger.debug("FMS is attached, auto-stopping recording");
//            Shuffleboard.stopRecording();
        }

        Shuffleboard.addEventMarker("Robot Disable", EventImportance.kNormal);

        Hardware.checkStickyFaults();
    }

    @Override
    public void disabledPeriodic() {
        Robot.limelight.setLeds(false);
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
        Robot.limelight.setLeds(true);

        Hardware.checkStickyFaults();

        if (DriverStation.getInstance().isFMSAttached()) {
            logger.debug("FMS is attached, auto-starting recording");
            Shuffleboard.setRecordingFileNameFormat(
                DriverStation.getInstance().getEventName() + "-" + DriverStation.getInstance()
                    .getMatchType() + "-" + DriverStation.getInstance().getMatchNumber()
                    + "-${date}-${time}");

//            Shuffleboard.startRecording();
        }

        Shuffleboard.addEventMarker("Autonomous Start", EventImportance.kNormal);

        if (elevator.getPosition() < 1) {
            elevator.setRaw(0);
        }

//        if (SmartDashboard.getBoolean("IsHatchPreload", true)) {
//            Robot.hatch.grab();
//        } else {
//            Robot.hatch.release();
//        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        OI.pointDriveCommand.start();
        Robot.limelight.setLeds(true);

        setMechanismBrakes(true);

        Robot.climber.raiseCylinder();

        Hardware.checkStickyFaults();

        if (DriverStation.getInstance().isFMSAttached()) {
//            Shuffleboard.startRecording();
        }

        Shuffleboard.addEventMarker("Teleop Start", EventImportance.kNormal);

        if (elevator.getPosition() < 1 && elevator.getCurrentCommand() == null) {
            elevator.setRaw(0);
        }
    }

    @Override
    public void teleopPeriodic() {
        if (SmartDashboard.getBoolean("EnableCompressor", true)) {
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
        } else {
            Hardware.compressor.stop();
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
