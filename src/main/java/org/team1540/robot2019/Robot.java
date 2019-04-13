package org.team1540.robot2019;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.apache.log4j.Level;
import org.apache.log4j.Logger;
import org.team1540.robot2019.commands.elevator.MoveElevatorToZero;
import org.team1540.robot2019.datastructures.threed.Transform3D;
import org.team1540.robot2019.datastructures.utils.UnitsUtils;
import org.team1540.robot2019.odometry.tankdrive.TankDriveOdometryAccumulatorRunnable;
import org.team1540.robot2019.subsystems.CargoMech;
import org.team1540.robot2019.subsystems.Climber;
import org.team1540.robot2019.subsystems.Drivetrain;
import org.team1540.robot2019.subsystems.Elevator;
import org.team1540.robot2019.subsystems.HatchMech;
import org.team1540.robot2019.subsystems.LEDs;
import org.team1540.robot2019.subsystems.Wrist;
import org.team1540.robot2019.utils.LastValidTransformTracker;
import org.team1540.robot2019.vision.deepspace.DeepSpaceVisionTargetLocalization;
import org.team1540.robot2019.wrappers.Limelight;
import org.team1540.rooster.util.SimpleCommand;

public class Robot extends TimedRobot {

    private static final Logger logger = Logger.getLogger(Robot.class);

    public static Drivetrain drivetrain;
    public static Elevator elevator;
    public static Wrist wrist;
    public static CargoMech cargoMech;
    public static HatchMech hatch;
    public static Climber climber;
    public static LEDs leds;

    public static boolean debugMode = false;

    boolean disableBrakes;
    private Timer brakeTimer = new Timer();

    public static TankDriveOdometryAccumulatorRunnable odometry;
    public static DeepSpaceVisionTargetLocalization deepSpaceVisionTargetLocalization;
    public static LastValidTransformTracker lastOdomToVisionTargetTracker;

    private UsbCamera cam = null;

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
        cargoMech = new CargoMech();
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

        PhineasShuffleboardTab.init();

        SmartDashboard.putBoolean("Debug Mode", SmartDashboard.getBoolean("Debug Mode", false));

        SmartDashboard.putBoolean("EnableCompressor", true);

//        Hardware.limelight.prepForDriverCam();

        String distanceGuessKey = "CameraPoseCalibration/DistanceGuessInInches";
        SmartDashboard.putNumber(distanceGuessKey, 0);
        Command estimatePitch = new SimpleCommand("Estimate Camera Pitch Command", () -> {
            double distanceEstimate = UnitsUtils.inchesToMeters(SmartDashboard.getNumber(distanceGuessKey, 0));
            if (distanceEstimate == 0) {
                logger.error("No distance estimate provided at networktables key: " + distanceGuessKey);
                return;
            }
            Double calibrationPitch = deepSpaceVisionTargetLocalization.estimateCorrectPitch(distanceEstimate, 1000, 0.001, true);
            if (calibrationPitch == null) {
                logger.error("calibrationPitch is null!");
            } else {
                logger.info("Pitch estimation successful: " + Math.toDegrees(calibrationPitch));
                SmartDashboard.putNumber("CameraPoseCalibration/PitchEstimate", Math.toDegrees(calibrationPitch));
            }
        });
        estimatePitch.setRunWhenDisabled(true);
        SmartDashboard.putData(estimatePitch);

        Command estimateYaw = new SimpleCommand("Estimate Camera Yaw Command", () -> {
            Double calibrationYaw = deepSpaceVisionTargetLocalization.estimateCorrectYaw(0, 1000, 0.001, true);
            if (calibrationYaw == null) {
                logger.error("calibrationYaw is null!");
            } else {
                logger.info("Yaw estimation successful: " + Math.toDegrees(calibrationYaw));
                SmartDashboard.putNumber("CameraPoseCalibration/YawEstimate", Math.toDegrees(calibrationYaw));
            }
        });
        estimateYaw.setRunWhenDisabled(true);
        SmartDashboard.putData(estimateYaw);

        Command calibrateCamera = new CommandGroup("Calibrate Camera Command") {{
            addSequential(estimatePitch);
            addSequential(estimateYaw);
        }};
        calibrateCamera.setRunWhenDisabled(true);
        SmartDashboard.putData(calibrateCamera);

        SmartDashboard.putBoolean("EnableUSBCamera", false);

        double end = RobotController.getFPGATime() / 1000.0; // getFPGATime returns microseconds
        logger.info("Robot ready. Initialization took " + (end - start) + " ms");
    }

    @Override
    public void robotPeriodic() {
        Scheduler.getInstance().run();

        debugMode = SmartDashboard.getBoolean("Debug Mode", false);

        if (DriverStation.getInstance().isFMSAttached()) {
            Logger.getRootLogger().setLevel(Level.WARN);
        } else {
            Logger.getRootLogger().setLevel(Level.DEBUG);
        }

        Transform3D lastBaseLinkToVisionTarget = deepSpaceVisionTargetLocalization.getLastBaseLinkToVisionTarget();
        if (lastBaseLinkToVisionTarget != null) {
            SmartDashboard.putNumber("CameraPoseCalibration/DistanceToTargetOut", UnitsUtils.metersToInches(lastBaseLinkToVisionTarget.toTransform2D().getPositionVector().distance(Vector2D.ZERO)));
        }
        if (debugMode) {
            odometry.getOdomToBaseLink().toTransform2D().putToNetworkTable("Odometry/Debug");
            if (lastOdomToVisionTargetTracker.getTransform3D() != null) {
                lastOdomToVisionTargetTracker.getTransform3D().toTransform2D()
                    .putToNetworkTable("DeepSpaceVisionTargetLocalization/Debug/OdomToVisionTarget");

            }
            if (lastBaseLinkToVisionTarget != null) {
                lastBaseLinkToVisionTarget.toTransform2D()
                    .putToNetworkTable("DeepSpaceVisionTargetLocalization/Debug/BaseLinkToVisionTarget");
            }
        }

        if ((cam == null) && SmartDashboard.getBoolean("EnableUSBCamera", false)) {
            cam = CameraServer.getInstance().startAutomaticCapture("USBCamera", 0);
            cam.setResolution(128, 73);
            cam.setFPS(30);
        }
        SmartDashboard.putBoolean("EnableUSBCamera", false);

//        NetworkTableInstance.getDefault().flush();
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
        compressorPeriodic();
    }

    @Override
    public void teleopInit() {
        Hardware.limelight.setLeds(true);

        setMechanismBrakes(true);

        Robot.climber.raiseCylinder();

        Hardware.checkStickyFaults();

        new MoveElevatorToZero().start();
    }

    @Override
    public void teleopPeriodic() {
        compressorPeriodic();
    }

    private void compressorPeriodic() {
        if (SmartDashboard.getBoolean("EnableCompressor", true)) {
            if ((Robot.elevator.getPosition() > Tuning.elevatorTolerance) && (Robot.climber.getCurrentCommand() == null)) {
                logger.debug("Stopping compressor because elevator is up");
                Hardware.compressor.stop();
            } else {
                if (Hardware.returnPressureSensorValue() > 120) {
                    Hardware.compressor.stop();
                } else if (Hardware.returnPressureSensorValue() < 110) {
                    Hardware.compressor.start();
                }
            }
        } else {
            Hardware.compressor.stop();
        }
    }

    private void setMechanismBrakes(boolean b) {
        drivetrain.setBrake(b);
        elevator.setBrake(b);
        climber.setArmBrake(b);
        wrist.setBrake(b);
    }
}
