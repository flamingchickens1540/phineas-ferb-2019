package org.team1540.robot2019;

import org.team1540.rooster.preferencemanager.TuningClass;

@TuningClass("ph_")
public class Tuning {

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

  public static double driveOpenLoopRamp = 0;
  public static double driveClosedLoopRamp = 0;

  public static boolean invertDriveLeft = true;
  public static boolean invertDriveLeftSensor = false;
  public static boolean invertDriveRight = false;
  public static boolean invertDriveRightSensor = false;

  public static int driveCurrentLimit = 40;

  // elevator
  public static boolean invertElevatorA = true;
  public static boolean invertElevatorB = false;

  public static double elevatorUpPosition = 0; // TODO: add these
  public static double elevatorCargoShipPosition = 0;
  public static double elevatorDownPosition = 0;

  // (2.872986590827646 in sprocket diameter * pi) * (5:1 gear ratio)
  public static double inPerRotation = -1.805150714;

  public static double elevatorTolerance = 0.5; // inches
  public static double elevatorVelocityTolerance = 1; // inches per second
  public static double elevatorStaticFeedForward = 1.3143; // volts
  public static double elevatorZeroingThrottle = 0.1;

  public static double elevatorDelta = 28; // inches

  public static double elevatorP = 0;
  public static double elevatorI = 0;
  public static double elevatorD = 0;

  // wrist
  public static boolean wristInvertMotor = true;

  public static double wristDownTravelPwrThrot = 0.7;
  public static double wristDownTravelBrakeThrot = 0.0;
  public static double wristUpTravelThrot = 0.55;

  // intake
  public static boolean intakeInvertTop = true;
  public static boolean intakeInvertBtm = false;

  public static double intakeIntakeSpeedTop = 1;
  public static double intakeIntakeSpeedBtm = 1;

  public static double intakeEjectSpeedTop = 1;
  public static double intakeEjectSpeedBtm = 1;

  public static double intakeTimeout = 5;

  public static double intakeEjectTime = 2;

  // hatch mech
  public static double hatchGetTime = 0.3; // DELETE IF NOT USING COMMAND
  public static double hatchPlaceTime = 0.3; // DELETE IF NOT USING COMMAND


  public static double pressureSensorVoltageScaleFactor;

  // auto-lineup
  public static double drivetrainRadius = 0.305; // This is for pandora

  public static long drivetrainUDPTimeout = 500;

  public static double LIMELIGHT_HORIZONTAL_FOV = Math.toRadians(59.6);
  public static double LIMELIGHT_VERTICAL_FOV = Math.toRadians(45.7);
}
