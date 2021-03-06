package org.team1540.robot2019;

public class Tuning {

    // general
    public static boolean isComp = true;

    // drive
    public static double driveVelocityP = 3;
    public static double driveVelocityI = 0.02;
    public static double driveVelocityD = 0;
    public static double driveVelocityF = 0;

    public static double drivePositionP = 0;
    public static double drivePositionI = 0;
    public static double drivePositionD = 0;
    public static double drivePositionF = 0;

    public static double driveThrottleExponent = 3;
    public static double driveSoftTurnExponent = 3;
    public static double driveHardTurnExponent = 1;

    public static double driveDeadzone = 0.2;

    // this unit is [-1,1] throttle per 20ms, which is kind of terrible but makes for nicer drive code
    // divide 0.02 by this to get seconds from neutral to full throttle
    public static double driveControlRampUp = .2;
    public static double driveControlRampDown = .2;

    public static double driveOpenLoopRamp = 0.18725;
    public static double driveClosedLoopRamp = 0;

    public static boolean invertDriveLeft = true;
    public static boolean invertDriveLeftSensor = !isComp;
    public static boolean invertDriveRight = false;
    public static boolean invertDriveRightSensor = !isComp;
    public static double drivetrainTicksPerMeter = 1017.3;
    public static double driveKV = 0.2625;
    public static double driveVIntercept = 0.049583333;
    public static double driveMaxVel = 3.620634921;

    public static int driveCurrentLimit = 40;

    public static double driveTestTime = 2;
    public static double driveTestWait = 2;
    public static double driveTestMotorThrot = 1;
    public static double driveTestStopTolerance = 0.1;

    // elevator
    public static boolean invertElevatorA = true;
    public static boolean invertElevatorB = false;

    public static double elevatorUpPosition = 28;
    public static double elevatorCargoShipPosition = 16;
    public static double elevatorClimbArmsBackPosition = 10;
    public static double elevatorHatchIntakePosition;
    public static double elevatorClimbPosition = 18;
    public static double elevatorLoadingStationPosition = 18;
    public static double elevatorDuringClimbPosition = 10;

    // (2.872986590827646 in sprocket diameter * pi) * (5:1 gear ratio)
    public static double elevatorInPerRotation = .929926125;

    public static double elevatorTolerance = 1; // inches
    public static double elevatorVelocityTolerance = 1; // inches per second
    public static double elevatorStaticFeedForward = 0.64; // volts
    public static double elevatorZeroingThrottle = 0.1;

    public static double elevatorDelta = 28; // inches

    public static double elevatorP = 0.1;
    public static double elevatorI = 0;
    public static double elevatorD = 5;

    public static double elevatorMaxCurrDiscrepancy = 10;
    public static double elevatorTestThrottle = 1;
    public static double elevatorTestTime = 0.1;
    public static double elevatorTestVelocityThresh = 1;

    // wrist
    public static boolean wristInvertMotor = true;

    public static double wristDownTravelPwrThrot;
    public static double wristDownTravelBrakeThrot;
    // what's uptravel
    public static double wristUpTravelThrot = 0.7;
    public static double wristHoldThrot = 0.08;
    public static double wristLowerTimeout = 1;

    // cargoMech
    public static boolean invertCargoRollerTop = false;
    public static boolean invertCargoRollerBottom = false;

    public static double cargoIntakeSpeedTop = 1;
    public static double cargoIntakeSpeedBtm = 1;

    public static double cargoEjectSpeedTop = 1;
    public static double cargoEjectSpeedBtm = 1;

    public static double cargoIntakeTimeout = 5;

    public static double cargoEjectTime = 1.3;

    // hatch mech
    public static double hatchGrabWaitTime = 1;
    public static double hatchExtendWaitTime = 0.2;
    public static double hatchReleaseWaitTime = 0.2;
    public static double hatchPrepFloorWaitTime = 0.5;
    public static double hatchDownWaitTime = 0.3;

    // climber
    public static double climberArmSpeed = 10000;
    public static double climberArmHoldSpeed = 0;
    public static double climberP = 0.5;
    public static double climberI = 0;
    public static double climberD = 0;
    public static double climberF = 0.1364;

    public static int climberMaxVel = 7500;
    public static int climberMaxAcc = 10000;

    public static double climberGyroP = 0.05;
    public static double climberGyroI = 0;
    public static double climberGyroD = 0;

    public static double climberArmsFwdLimit = 67000;
    public static double climberStartPosLevel2 = 42000;
    //    public static double climberStartPosLevel2 = 50000;
    public static double climberStartPosLevel3 = 30000;
    public static double climberBackPos = 0;

    public static double climberGyroFF = 0.2;
    public static double climberUpPosition = 53000; // arms

    public static double climberTolerance = 1000;

    public static double axisButtonThreshold = 0.4;

    // auto-lineup
    public static double drivetrainRadiusMeters = 0.305; // This is for pandora (Seems to work... hmmm) TODO: Change this

    public static long drivetrainUDPTimeout = 500;

    public static double drivetrainMaxVelocity = 400;

    public static double ledTime = 5;
    public static double ledStrobeTime = 0.2;

    static {
        //noinspection ConstantConditions
        if (Tuning.isComp) {
            wristDownTravelPwrThrot = 1.0;
            wristDownTravelBrakeThrot = 0.75;
            elevatorHatchIntakePosition = 0;
        } else {
            wristDownTravelPwrThrot = 0.7;
            wristDownTravelBrakeThrot = 0.25;
            elevatorHatchIntakePosition = 1.4;
        }
    }
}
