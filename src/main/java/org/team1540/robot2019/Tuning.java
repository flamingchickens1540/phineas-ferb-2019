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
  public static double elevatorUpSpeed = 1;
  public static double elevatorDownSpeed = 1;

  public static boolean invertElevatorA = true;
  public static boolean invertElevatorB = false;

  public static double elevatorUpPosition = 0; // TODO: add these
  public static double elevatorCargoShipPosition = 0;
  public static double elevatorDownPosition = 0;

  // (2.872986590827646 in sprocket diameter * pi) / (5:1 gear ratio)
  public static double elevatorRotationsPerIn = 1.805150714;
  public static double elevatorMaxAccelUp = 600; // in/s^2

  public static double elevatorMaxVel = 130; // in/s^2 // TODO: rough guess based on motor specs
  public static double elevatorMaxAccelDown = 1346; // in/s^2/ / TODO: rough guess based on motor specs
  // range, in inches, inside of which we use standard PID instead of a trapezoidal curve
  public static double elevatorMinTrapezoidalRange = 1; // inches
  // range, in inches, inside of which we engage the brake
  public static double elevatorHoldThrottle = 0.4; // TODO: rough guess based on motor specs
  // volts per inch per sec
  public static double elevatorVelCoeff = 0.092307692; // TODO: rough guess based on motor specs
  // volts per inch per sec^2
  public static double elevatorAccelCoeff = 0.012120702; // TODO: rough guess based on motor specs

  public static double elevatorDelta = 28; // inches

  // wrist
  public static boolean wristInvertMotor = false;

  public static double wristDownTravelPwrThrot = 1;
  public static double wristDownTravelBrakeThrot = 0.2;
  public static double wristUpTravelThrot = 1;

  // intake
  public static boolean intakeInvertTop = false;
  public static boolean intakeInvertBtm = true;

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
}
