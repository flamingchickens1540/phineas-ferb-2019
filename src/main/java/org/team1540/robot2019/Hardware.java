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
import org.team1540.robot2019.wrappers.NavX;
import org.team1540.rooster.wrappers.ChickenController;
import org.team1540.rooster.wrappers.ChickenTalon;
import org.team1540.rooster.wrappers.ChickenVictor;

/**
 * This is my fancy replacement for the RobotMap class. Now instead of centralizing motor numbers, I centralize the motors themselves. This also means we don't have to redo loads of config when making
 * alternative robots or funky testing code.
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

    public static DigitalInput elevatorLimitSensor;

    public static DigitalInput wristMidSwitch;
    public static DigitalInput wristBtmSwitch;


    // solenoid on is wrist extended/down
    public static ChickenController wristMotor;


    // positive setpoint is outtaking
    public static ChickenController intakeTop;
    public static ChickenController intakeBtm;

    public static DigitalInput intakeSensor;

    public static Solenoid hatchSlide;
    public static Solenoid hatchGrabber;

    public static ChickenTalon climberArmLeft;
    public static ChickenController climberArmRight;

    public static DoubleSolenoid climberCylinder;

    public static Solenoid blueLEDs;
    public static Solenoid redLEDs;

    public static AnalogInput pressureSensor;

    public static NavX navx;

    // initialized statically as there's literally no scenario where the PDP wouldn't be connected
    public static PowerDistributionPanel pdp = new PowerDistributionPanel();

    public static Compressor compressor;

    static void initAll() {
        logger.info("Initializing robot hardware...");
        double start = RobotController.getFPGATime() / 1000.0; // getFPGATime returns microseconds

        initDrive();
        initElevator();
        initWrist();
        initIntake();
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

        elevatorLimitSensor = new DigitalInput(RobotMap.ELEVATOR_LIMIT_SENSOR);

        double end = RobotController.getFPGATime() / 1000.0;
        logger.info("Initialized elevator in " + (end - start) + " ms");
    }

    public static void initWrist() {
        logger.info("Initializing wrist...");
        double start = RobotController.getFPGATime() / 1000.0; // getFPGATime returns microseconds

        wristMotor = createController(RobotMap.INTAKE_WRIST);

        wristMotor.setInverted(Tuning.wristInvertMotor);
        wristMotor.setBrake(true);

        wristMotor.configVoltageCompSaturation(12);
        wristMotor.enableVoltageCompensation(true);

        wristMidSwitch = new DigitalInput(RobotMap.WRIST_MID_SW);
        wristBtmSwitch = new DigitalInput(RobotMap.WRIST_BTM_SW);

        double end = RobotController.getFPGATime() / 1000.0;
        logger.info("Initialized wrist in " + (end - start) + " ms");
    }

    public static void initIntake() {
        logger.info("Initializing cargoMechanism...");
        double start = RobotController.getFPGATime() / 1000.0; // getFPGATime returns microseconds

        intakeTop = createController(RobotMap.INTAKE_TOP);
        intakeBtm = createController(RobotMap.INTAKE_BTM);

        intakeTop.setInverted(Tuning.intakeInvertTop);
        intakeBtm.setInverted(Tuning.intakeInvertBtm);

        intakeTop.setBrake(true);
        intakeBtm.setBrake(true);

        intakeTop.configVoltageCompSaturation(12);
        intakeTop.enableVoltageCompensation(true);

        intakeBtm.configVoltageCompSaturation(12);
        intakeBtm.enableVoltageCompensation(true);

        intakeSensor = new DigitalInput(RobotMap.INTAKE_SENSOR);

        double end = RobotController.getFPGATime() / 1000.0;
        logger.info("Initialized cargoMechanism in " + (end - start) + " ms");
    }

    public static void initHatchMech() {
        logger.info("Initializing hatch mech...");
        double start = RobotController.getFPGATime() / 1000.0; // getFPGATime returns microseconds

        hatchSlide = new Solenoid(RobotMap.HATCH_SLIDE);
        hatchGrabber = new Solenoid(RobotMap.HATCH_GRABBER);

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

        blueLEDs = new Solenoid(RobotMap.LED_BLUE);
        redLEDs = new Solenoid(RobotMap.LED_RED);

        double end = RobotController.getFPGATime() / 1000.0;
        logger.info("Initialized LEDs in " + (end - start) + " ms");
    }

    public static void checkStickyFaults() {
        StickyFaultsProcessor.processStickyFaults("Drivetrain", "left A", driveLeftMotorA);
        StickyFaultsProcessor.processStickyFaults("Drivetrain", "left B", driveLeftMotorB);
        StickyFaultsProcessor.processStickyFaults("Drivetrain", "left C", driveLeftMotorC);
        StickyFaultsProcessor.processStickyFaults("Drivetrain", "right A", driveRightMotorA);
        StickyFaultsProcessor.processStickyFaults("Drivetrain", "right B", driveRightMotorB);
        StickyFaultsProcessor.processStickyFaults("Drivetrain", "right C", driveRightMotorC);

        StickyFaultsProcessor.processStickyFaults("Wrist", "motor", wristMotor);

        StickyFaultsProcessor.processStickyFaults("Intake", "top", intakeTop);
        StickyFaultsProcessor.processStickyFaults("Intake", "bottom", intakeBtm);
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

    public static double getIntakeWristCurrent() {
        if (wristMotor instanceof ChickenTalon) {
            return ((ChickenTalon) wristMotor).getOutputCurrent();
        } else {
            return pdp.getCurrent(RobotMap.PDP_INTAKE_WRIST);
        }
    }

    public static double getIntakeTopCurrent() {
        if (intakeTop instanceof ChickenTalon) {
            return ((ChickenTalon) intakeTop).getOutputCurrent();
        } else {
            return pdp.getCurrent(RobotMap.PDP_INTAKE_TOP);
        }
    }

    public static double getIntakeBtmCurrent() {
        if (intakeBtm instanceof ChickenTalon) {
            return ((ChickenTalon) intakeBtm).getOutputCurrent();
        } else {
            return pdp.getCurrent(RobotMap.PDP_INTAKE_BTM);
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
