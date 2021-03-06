package org.team1540.robot2019;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Solenoid;
import org.apache.log4j.Logger;
import org.team1540.robot2019.utils.StickyFaultsUtils;
import org.team1540.robot2019.wrappers.Limelight;
import org.team1540.robot2019.wrappers.NavX;
import org.team1540.rooster.wrappers.ChickenController;
import org.team1540.rooster.wrappers.ChickenTalon;
import org.team1540.rooster.wrappers.ChickenVictor;

/**
 * This is my fancy replacement for the RobotMap class. Now instead of centralizing motor numbers, I centralize the motors themselves. This also means we don't have to redo loads of config when making alternative robots or funky temporary
 * code.
 */
public class Hardware {

    private static final Logger logger = Logger.getLogger(Hardware.class);

    private static boolean hasLoggedCompBot = false;

    public static final int DRIVE_POSITION_SLOT_IDX = 0;
    public static final int DRIVE_VELOCITY_SLOT_IDX = 1;

    // these aren't final for initialization but don't change them mkay

    public static ChickenTalon driveLeftMotorA;
    public static ChickenController driveLeftMotorB;
    public static ChickenController driveLeftMotorC;
    public static ChickenController[] driveLeftMotors;
    public static ChickenTalon driveRightMotorA;
    public static ChickenController driveRightMotorB;
    public static ChickenController driveRightMotorC;
    public static ChickenController[] driveRightMotors;
    public static ChickenController[] driveMotorAll;
    public static ChickenTalon[] driveMotorMasters;


    // positive setpoint is up
    public static CANSparkMax elevatorA;
    public static CANSparkMax elevatorB;

    public static DigitalInput wristMidSwitch;
    public static DigitalInput wristBtmSwitch;

    public static DigitalInput hatchExtendSwitch;

    // solenoid on is wrist extended/down
    public static ChickenController wristMotor;


    // positive setpoint is outtaking
    public static ChickenTalon cargoRollerTop;
    public static ChickenController cargoRollerBottom;

    public static DigitalInput cargoIntakeSensor;

    public static Solenoid hatchSlide;
    public static Solenoid hatchGrabber;

    public static ChickenTalon climberArmLeft;
    public static ChickenController climberArmRight;

    public static DoubleSolenoid climberCylinder;

    public static Solenoid redLEDs;
    public static Solenoid greenLEDs;
    public static Solenoid blueLEDs;

    public static AnalogInput pressureSensor;

    public static NavX navx;

    public static Limelight limelight;

    // initialized statically as there's literally no scenario where the PDP wouldn't be connected
    public static PowerDistributionPanel pdp = new PowerDistributionPanel();

    public static Compressor compressor;

    // temp
    static double returnPressureSensorValue() {
        if (Hardware.pressureSensor == null) {
            return 0;
        }
        return 50 * (Hardware.pressureSensor.getVoltage() - 0.5);
    }

    static void initAll() {
        logger.info("Initializing robot hardware...");
        double start = RobotController.getFPGATime() / 1000.0; // getFPGATime returns microseconds

        initDrive();
        initElevator();
        initWrist();
        initCargoMech();
        initHatchMech();
        initClimber();
        initCompressor();
        initPressureSensor();
        initNavX();
        initLEDs();

        double end = RobotController.getFPGATime() / 1000.0;
        logger.info("Initialized robot hardware in " + (end - start) + " ms");
    }

    public static void initDrive() {
        logger.info("Initializing drive...");
        double start = RobotController.getFPGATime() / 1000.0; // getFPGATime returns microseconds

        driveLeftMotorA = new ChickenTalon(RobotMap.DRIVE_LEFT_A);
        driveLeftMotorB = createController(RobotMap.DRIVE_LEFT_B);
        driveLeftMotorC = createController(RobotMap.DRIVE_LEFT_C);

        driveRightMotorA = new ChickenTalon(RobotMap.DRIVE_RIGHT_A);
        driveRightMotorB = createController(RobotMap.DRIVE_RIGHT_B);
        driveRightMotorC = createController(RobotMap.DRIVE_RIGHT_C);

        driveMotorAll = new ChickenController[]{driveLeftMotorA, driveLeftMotorB, driveLeftMotorC,
            driveRightMotorA, driveRightMotorB, driveRightMotorC};
        driveMotorMasters = new ChickenTalon[]{driveLeftMotorA, driveRightMotorA};
        driveLeftMotors = new ChickenController[]{driveLeftMotorA, driveLeftMotorB, driveLeftMotorC};
        driveRightMotors = new ChickenController[]{driveRightMotorA, driveRightMotorB,
            driveRightMotorC};

        for (ChickenController talon : driveMotorAll) {
            talon.setBrake(false);
            talon.configVoltageCompSaturation(12);
            talon.enableVoltageCompensation(true);

            talon.configPeakOutputForward(1);
            talon.configPeakOutputReverse(-1);
            talon.configOpenloopRamp(Tuning.driveOpenLoopRamp);
            talon.configForwardSoftLimitEnable(false);
            talon.configReverseSoftLimitEnable(false);
            talon.overrideLimitSwitchesEnable(false);
        }

        for (ChickenTalon talon : driveMotorMasters) {
            talon.setBrake(true);

            talon.config_kP(DRIVE_POSITION_SLOT_IDX, Tuning.drivePositionP);
            talon.config_kI(DRIVE_POSITION_SLOT_IDX, Tuning.drivePositionI);
            talon.config_kD(DRIVE_POSITION_SLOT_IDX, Tuning.drivePositionD);
            talon.config_kF(DRIVE_POSITION_SLOT_IDX, Tuning.drivePositionF);
            talon.config_kP(DRIVE_VELOCITY_SLOT_IDX, Tuning.driveVelocityP);
            talon.config_kI(DRIVE_VELOCITY_SLOT_IDX, Tuning.driveVelocityI);
            talon.config_kD(DRIVE_VELOCITY_SLOT_IDX, Tuning.driveVelocityD);
            talon.config_kF(DRIVE_VELOCITY_SLOT_IDX, Tuning.driveVelocityF);

            // at the moment, this hard caps to driveCurrentLimit; we might implement peak limiting
            // instead
            talon.configPeakCurrentLimit(0);
            talon.configContinuousCurrentLimit(Tuning.driveCurrentLimit);
        }

        for (ChickenController talon : driveLeftMotors) {
            talon.setInverted(Tuning.invertDriveLeft);
        }

        for (ChickenController talon : driveRightMotors) {
            talon.setInverted(Tuning.invertDriveRight);
        }

        driveLeftMotorA.setSensorPhase(Tuning.invertDriveLeftSensor);
        driveRightMotorA.setSensorPhase(Tuning.invertDriveRightSensor);

        driveLeftMotorB.follow(driveLeftMotorA);
        driveLeftMotorC.follow(driveLeftMotorA);
        driveRightMotorB.follow(driveRightMotorA);
        driveRightMotorC.follow(driveRightMotorA);

        double end = RobotController.getFPGATime() / 1000.0;
        logger.info("Initialized drive in " + (end - start) + " ms");
    }

    public static void initElevator() {
        logger.info("Initializing elevator...");
        double start = RobotController.getFPGATime() / 1000.0; // getFPGATime returns microseconds

        elevatorA = new CANSparkMax(RobotMap.ELEVATOR_L, MotorType.kBrushless);
        elevatorB = new CANSparkMax(RobotMap.ELEVATOR_R, MotorType.kBrushless);

        elevatorA.setIdleMode(IdleMode.kBrake);
        elevatorB.setIdleMode(IdleMode.kBrake);

        elevatorA.setInverted(Tuning.invertElevatorA);
        elevatorB.follow(elevatorA, Tuning.invertElevatorB);

        elevatorA.getPIDController().setP(Tuning.elevatorP);
        elevatorA.getPIDController().setI(Tuning.elevatorI);
        elevatorA.getPIDController().setD(Tuning.elevatorD);
        elevatorA.getPIDController().setOutputRange(-1, 1);

        double end = RobotController.getFPGATime() / 1000.0;
        logger.info("Initialized elevator in " + (end - start) + " ms");
    }

    public static void initElevatorIndependent() {
        logger.info("Initializing elevator in independent mode...");
        double start = RobotController.getFPGATime() / 1000.0; // getFPGATime returns microseconds

        elevatorA = new CANSparkMax(RobotMap.ELEVATOR_L, MotorType.kBrushless);
        elevatorB = new CANSparkMax(RobotMap.ELEVATOR_R, MotorType.kBrushless);

        elevatorA.setIdleMode(IdleMode.kBrake);
        elevatorB.setIdleMode(IdleMode.kBrake);

        elevatorA.setInverted(Tuning.invertElevatorA);

        double end = RobotController.getFPGATime() / 1000.0;
        logger.info("Initialized elevator in " + (end - start) + " ms");
    }

    public static void initWrist() {
        logger.info("Initializing wrist...");
        double start = RobotController.getFPGATime() / 1000.0; // getFPGATime returns microseconds

        wristMotor = createController(RobotMap.WRIST_MOTOR);

        wristMotor.setInverted(Tuning.wristInvertMotor);
        wristMotor.setBrake(true);

        wristMotor.configVoltageCompSaturation(12);
        wristMotor.enableVoltageCompensation(true);

        wristMidSwitch = new DigitalInput(RobotMap.WRIST_MID_SW);
        wristBtmSwitch = new DigitalInput(RobotMap.WRIST_BTM_SW);

        double end = RobotController.getFPGATime() / 1000.0;
        logger.info("Initialized wrist in " + (end - start) + " ms");
    }

    public static void initCargoMech() {
        logger.info("Initializing cargo mech...");
        double start = RobotController.getFPGATime() / 1000.0; // getFPGATime returns microseconds

        cargoRollerTop = new ChickenTalon(RobotMap.CARGO_ROLLER_TOP);
        cargoRollerTop.configPeakCurrentLimit(50);
        cargoRollerBottom = createController(RobotMap.CARGO_ROLLER_BOTTOM);

        cargoRollerTop.setInverted(Tuning.invertCargoRollerTop);
        cargoRollerBottom.setInverted(Tuning.invertCargoRollerBottom);
        cargoRollerTop.setBrake(true);
        cargoRollerBottom.setBrake(true);

        cargoRollerTop.configVoltageCompSaturation(12);
        cargoRollerTop.enableVoltageCompensation(true);

        cargoRollerBottom.configVoltageCompSaturation(12);
        cargoRollerBottom.enableVoltageCompensation(true);

        cargoIntakeSensor = new DigitalInput(RobotMap.CARGO_INTAKE_SENSOR);

        double end = RobotController.getFPGATime() / 1000.0;
        logger.info("Initialized cargoMech in " + (end - start) + " ms");
    }

    public static void initHatchMech() {
        logger.info("Initializing hatch mech...");
        double start = RobotController.getFPGATime() / 1000.0; // getFPGATime returns microseconds

        hatchSlide = new Solenoid(RobotMap.HATCH_SLIDE);
        hatchGrabber = new Solenoid(RobotMap.HATCH_GRABBER);

        hatchExtendSwitch = new DigitalInput(RobotMap.HATCH_EXTEND_SW);

        double end = RobotController.getFPGATime() / 1000.0;
        logger.info("Initialized hatch mech in " + (end - start) + " ms");
    }

    public static void initClimber() {
        logger.info("Initializing climber...");
        double start = RobotController.getFPGATime() / 1000.0; // getFPGATime returns microseconds

        climberArmLeft = new ChickenTalon(RobotMap.CLIMBER_ARM_L);
        climberArmRight = createController(RobotMap.CLIMBER_ARM_R);

        climberArmLeft.setBrake(true);
        climberArmRight.setBrake(true);

        climberArmLeft.setInverted(true);
        climberArmRight.setInverted(true);

        climberArmLeft.configPeakOutputForward(1);
        climberArmLeft.configPeakOutputReverse(-1);

        climberArmLeft.configVoltageCompSaturation(12);
        climberArmLeft.enableVoltageCompensation(true);
        climberArmRight.configVoltageCompSaturation(12);
        climberArmRight.enableVoltageCompensation(true);

        climberArmRight.follow(climberArmLeft);

        climberArmLeft.config_kP(0, Tuning.climberP);
        climberArmLeft.config_kI(0, Tuning.climberI);
        climberArmLeft.config_kD(0, Tuning.climberD);
        climberArmLeft.config_kF(0, Tuning.climberF);
        climberArmLeft.configMotionCruiseVelocity(Tuning.climberMaxVel);
        climberArmLeft.configMotionAcceleration(Tuning.climberMaxAcc);
        climberArmLeft.setSensorPhase(true);
        climberArmLeft.setSelectedSensorPosition(0);

        climberCylinder = new DoubleSolenoid(RobotMap.CLIMBER_CYLINDER_1, RobotMap.CLIMBER_CYLINDER_2);

        double end = RobotController.getFPGATime() / 1000.0;
        logger.info("Initialized climber in " + (end - start) + " ms");
    }

    public static void initCompressor() {
        logger.info("Initializing compressor...");
        double start = RobotController.getFPGATime() / 1000.0; // getFPGATime returns microseconds

        compressor = new Compressor();

        double end = RobotController.getFPGATime() / 1000.0;
        logger.info("Initialized compressor in " + (end - start) + " ms");
    }

    public static void initPressureSensor() {
        logger.info("Initializing pressure sensor...");
        double start = RobotController.getFPGATime() / 1000.0; // getFPGATime returns microseconds

        pressureSensor = new AnalogInput(RobotMap.PRESSURE_SENSOR);

        double end = RobotController.getFPGATime() / 1000.0;
        logger.info("Initialized pressure sensor in " + (end - start) + " ms");
    }

    public static void initNavX() {
        logger.info("Initializing NavX-MXP...");
        double start = RobotController.getFPGATime() / 1000.0; // getFPGATime returns microseconds

        navx = new NavX(RobotMap.NAVX);

        double end = RobotController.getFPGATime() / 1000.0;
        logger.info("Initialized NavX-MXP in " + (end - start) + " ms");
    }

    public static void initLEDs() {
        logger.info("Initializing LEDs...");
        double start = RobotController.getFPGATime() / 1000.0; // getFPGATime returns microseconds

        redLEDs = new Solenoid(RobotMap.LED_RED);
        greenLEDs = new Solenoid(RobotMap.LED_GREEN);
        blueLEDs = new Solenoid(RobotMap.LED_BLUE);

        double end = RobotController.getFPGATime() / 1000.0;
        logger.info("Initialized LEDs in " + (end - start) + " ms");
    }

    public static void checkStickyFaults() {
        StickyFaultsUtils.processStickyFaults("Drivetrain", "left A", driveLeftMotorA);
        StickyFaultsUtils.processStickyFaults("Drivetrain", "left B", driveLeftMotorB);
        StickyFaultsUtils.processStickyFaults("Drivetrain", "left C", driveLeftMotorC);
        StickyFaultsUtils.processStickyFaults("Drivetrain", "right A", driveRightMotorA);
        StickyFaultsUtils.processStickyFaults("Drivetrain", "right B", driveRightMotorB);
        StickyFaultsUtils.processStickyFaults("Drivetrain", "right C", driveRightMotorC);

        StickyFaultsUtils.processStickyFaults("Wrist", "motor", wristMotor);

        StickyFaultsUtils.processStickyFaults("CargoMech", "top", cargoRollerTop);
        StickyFaultsUtils.processStickyFaults("CargoMech", "bottom", cargoRollerBottom);
    }

    public static ChickenController createController(int id) {
        if (!hasLoggedCompBot) {
            if (Tuning.isComp) {
                logger.info("Competition robot detected, initializing motors as Victor SPXs");
            } else {
                logger.info("Practice robot detected, initializing motors as Talon SRXs");
            }
            hasLoggedCompBot = true;
        }

        return Tuning.isComp ? new ChickenVictor(id) : new ChickenTalon(id);
    }

    public static double getDriveLeftACurrent() {
        return driveLeftMotorA.getOutputCurrent();
    }

    public static double getDriveLeftBCurrent() {
        if (driveLeftMotorB instanceof ChickenTalon) {
            return ((ChickenTalon) driveLeftMotorB).getOutputCurrent();
        } else {
            return pdp.getCurrent(RobotMap.PDP_DRIVE_LEFT_B);
        }
    }

    public static double getDriveLeftCCurrent() {
        if (driveLeftMotorC instanceof ChickenTalon) {
            return ((ChickenTalon) driveLeftMotorC).getOutputCurrent();
        } else {
            return pdp.getCurrent(RobotMap.PDP_DRIVE_LEFT_C);
        }
    }

    public static double getDriveRightACurrent() {
        return driveRightMotorA.getOutputCurrent();
    }

    public static double getDriveRightBCurrent() {
        if (driveRightMotorB instanceof ChickenTalon) {
            return ((ChickenTalon) driveRightMotorB).getOutputCurrent();
        } else {
            return pdp.getCurrent(RobotMap.PDP_DRIVE_RIGHT_B);
        }
    }

    public static double getDriveRightCCurrent() {
        if (driveRightMotorC instanceof ChickenTalon) {
            return ((ChickenTalon) driveRightMotorC).getOutputCurrent();
        } else {
            return pdp.getCurrent(RobotMap.PDP_DRIVE_RIGHT_C);
        }
    }

    public static double getWristMotorCurrent() {
        if (wristMotor instanceof ChickenTalon) {
            return ((ChickenTalon) wristMotor).getOutputCurrent();
        } else {
            return pdp.getCurrent(RobotMap.PDP_WRIST_MOTOR);
        }
    }

    public static double getCargoMechTopCurrent() {
        return cargoRollerTop.getOutputCurrent();
    }

    public static double getCargoMechBtmCurrent() {
        if (cargoRollerBottom instanceof ChickenTalon) {
            return ((ChickenTalon) cargoRollerBottom).getOutputCurrent();
        } else {
            return pdp.getCurrent(RobotMap.PDP_CARGO_MECH_BTM);
        }
    }

    public static double getClimberLCurrent() {
        return climberArmLeft.getOutputCurrent();
    }

    public static double getClimberRCurrent() {
        if (climberArmRight instanceof ChickenTalon) {
            return ((ChickenTalon) climberArmRight).getOutputCurrent();
        } else {
            return pdp.getCurrent(RobotMap.PDP_CLIMBER_ARM_R);
        }
    }
}
