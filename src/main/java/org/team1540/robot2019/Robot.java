package org.team1540.robot2019;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.apache.log4j.Level;
import org.apache.log4j.Logger;
import org.team1540.robot2019.datastructures.threed.Transform3D;
import org.team1540.robot2019.odometry.tankdrive.TankDriveOdometryAccumulatorRunnable;
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

    public static TankDriveOdometryAccumulatorRunnable odometry;
    public static DeepSpaceVisionTargetLocalization deepSpaceVisionTargetLocalization;
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

        odometry = new TankDriveOdometryAccumulatorRunnable(
            drivetrain::getLeftPositionMeters,
            drivetrain::getRightPositionMeters,
            Hardware.navx::getAngleRadians,
            0.01
        );

        Hardware.limelight = new Limelight("limelight-a",
            new Transform3D(RobotMap.CAM_X, RobotMap.CAM_Y, RobotMap.CAM_Z, RobotMap.CAM_ROLL,
                RobotMap.CAM_PITCH, RobotMap.CAM_YAW));
        lastOdomToVisionTargetTracker = new LastValidTransformTracker(odometry::getOdomToBaseLink);
        deepSpaceVisionTargetLocalization = new DeepSpaceVisionTargetLocalization(Hardware.limelight,
            RobotMap.HATCH_TARGET_HEIGHT, 0.05,
            lastOdomToVisionTargetTracker); // Doesn't have to be very frequent if things that use it also call update

        OI.init();

        ShuffleboardDisplay.init();

        // TODO: use shuffleboard properly
        SmartDashboard.putBoolean("IsHatchPreload", false);
        SmartDashboard.putBoolean("Debug Mode", false);

        SmartDashboard.putBoolean("EnableCompressor", true);

        Hardware.limelight.prepForDriverCam();

        SmartDashboard.setDefaultBoolean("EnableBackupCam", false);
        if (SmartDashboard.getBoolean("EnableBackupCam", false)) {
            UsbCamera cam = CameraServer.getInstance().startAutomaticCapture("backup cam", 0);
            cam.setResolution(128, 73);
            cam.setFPS(30);
        }

        SmartDashboard.putNumber("CalibrationDistance", 1);
        Command estimatePitch = new SimpleCommand("Estimate Camera Pitch", () -> {
            Double calibrationPitch = deepSpaceVisionTargetLocalization.estimateCorrectPitch(SmartDashboard.getNumber("CalibrationDistance", 0), 1000, 0.001);
            if (calibrationPitch == null) {
                logger.error("calibrationPitch is null!");
            } else {
                logger.info("Pitch estimation successful: " + calibrationPitch);
            }
        });
        estimatePitch.setRunWhenDisabled(true);
        SmartDashboard.putData(estimatePitch);

        Command estimateYaw = new SimpleCommand("Estimate Camera Yaw", () -> {
            Double calibrationPitch = deepSpaceVisionTargetLocalization.estimateCorrectYaw(0, 1000, 0.001);
            if (calibrationPitch == null) {
                logger.error("calibrationYaw is null!");
            } else {
                logger.info("Yaw estimation successful: " + calibrationPitch);
            }
        });
        estimateYaw.setRunWhenDisabled(true);
        SmartDashboard.putData(estimateYaw);

        double end = RobotController.getFPGATime() / 1000.0; // getFPGATime returns microseconds
        logger.info("Robot ready. Initialization took " + (end - start) + " ms");
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
        NetworkTableInstance.getDefault().flush();
    }

    @Override
    public void disabledInit() {
        Hardware.limelight.setLeds(false);
        logger.debug("Disabling mechanism brakes in 2 seconds...");
        brakeTimer.reset();
        brakeTimer.start();
        disableBrakes = true;

        Robot.hatch.retract();
        Robot.hatch.grab(); // otherwise we might flicker grab-release on enable

        Hardware.checkStickyFaults();
    }

    @Override
    public void disabledPeriodic() {
        if (brakeTimer.hasPeriodPassed(2) && disableBrakes) {
            brakeTimer.stop();
            setMechanismBrakes(false);

            logger.debug("Mechanism brakes disabled");

            disableBrakes = false;
        }
    }

    @Override
    public void autonomousInit() {
        setMechanismBrakes(true);
        Hardware.limelight.setLeds(true);

        Hardware.checkStickyFaults();

        if (elevator.getPosition() < 1) {
            elevator.setRaw(0);
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        OI.pointDriveCommand.start();
        Hardware.limelight.setLeds(true);

        setMechanismBrakes(true);

        Robot.climber.raiseCylinder();

        Hardware.checkStickyFaults();

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
