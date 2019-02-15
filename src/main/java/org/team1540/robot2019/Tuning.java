package org.team1540.robot2019;

import org.team1540.rooster.preferencemanager.TuningClass;

@TuningClass("ph_")
public class Tuning {

  // general
  public static boolean isComp = true;

  // drive
  public static double driveVelocityP = 0;
  public static double driveVelocityI = 0;
  public static double driveVelocityD = 0;
  public static double driveVelocityF = 0;

  public static double drivePositionP = 0;
  public static double drivePositionI = 0;
  public static double drivePositionD = 0;
  public static double drivePositionF = 0;

  public static double driveThrottleExponent = 3;
  public static double driveSoftTurnExponent = 3;
  public static double driveHardTurnExponent = 3;

  public static double driveDeadzone = 0.1;

  // this unit is [-1,1] throttle per 20ms, which is kind of terrible but makes for nicer drive code
  // divide 0.02 by this to get seconds from neutral to full throttle
  public static double driveControlRampUp = .2;
  public static double driveControlRampDown = .2;

  public static double driveOpenLoopRamp = 0.25;
  public static double driveClosedLoopRamp = 0;

  public static boolean invertDriveLeft = true;
  public static boolean invertDriveLeftSensor = false;
  public static boolean invertDriveRight = false;
  public static boolean invertDriveRightSensor = false;
  public static double driveTicksPerMeter = 1047.8;
  public static double driveKV = 0.25975;
  public static double driveVIntercept = 0.054083333;
  public static double driveMaxVel = 3.645171639;

  public static int driveCurrentLimit = 40;

  // elevator
  public static boolean invertElevatorA = true;
  public static boolean invertElevatorB = false;

  public static double elevatorUpPosition = 28;
  public static double elevatorCargoShipPosition = 14;
  public static double elevatorDownPosition = 0;
  public static double elevatorClimbPosition = 14;

  // (2.872986590827646 in sprocket diameter * pi) * (5:1 gear ratio)
  public static double elevatorInPerRotation = .929926125;

  public static double elevatorTolerance = 0.5; // inches
  public static double elevatorVelocityTolerance = 1; // inches per second
  public static double elevatorStaticFeedForward = 1.3143; // volts
  public static double elevatorZeroingThrottle = 0.1;

  public static double elevatorDelta = 28; // inches

  public static double elevatorP = 0.1;
  public static double elevatorI = 0;
  public static double elevatorD = 5;

  // wrist
  public static boolean wristInvertMotor = true;

  public static double wristDownTravelPwrThrot = 0.7;
  public static double wristDownTravelBrakeThrot = 0.0;
  // what's uptravel
  public static double wristUpTravelThrot = 0.55;
  public static double wristHoldThrot = 0.1;
  public static double wristLowerTimeout = 1;

  // intake
  public static boolean intakeInvertTop = false;
  public static boolean intakeInvertBtm = false;

  public static double intakeIntakeSpeedTop = 1;
  public static double intakeIntakeSpeedBtm = 1;

  public static double intakeEjectSpeedTop = 1;
  public static double intakeEjectSpeedBtm = 1;

  public static double intakeTimeout = 5;

  public static double intakeEjectTime = 1;

  // hatch mech
  public static double hatchGetTime = 0.6;
  public static double hatchPlaceTime = 0.6;

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
  public static double climberGyroStartPos = 30000;
  public static double climberGyroFF = 0.5;
  public static double climberUpPosition = 53000; // arms

  public static double climberTolerance = 1000;
}
