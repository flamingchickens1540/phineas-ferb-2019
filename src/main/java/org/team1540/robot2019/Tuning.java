package org.team1540.robot2019;

import org.team1540.rooster.preferencemanager.TuningClass;

@TuningClass("phineas_")
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

  public static double driveControlRamp = .2;
  public static double driveOpenLoopRamp = 0;
  public static double driveClosedLoopRamp = 0;

  public static boolean invertDriveLeft = false;
  public static boolean invertDriveLeftSensor = false;
  public static boolean invertDriveRight = true;
  public static boolean invertDriveRightSensor = false;

  public static int driveCurrentLimit = 40;

  // elevator
  public static double elevatorUpSpeed = 1;
  public static double elevatorDownSpeed = 1;

  public static boolean invertElevatorA = false;
  public static boolean invertElevatorB = false;

  // intake
  public static boolean intakeInvertTop = false;
  public static boolean intakeInvertBtm = false;

  public static double intakeIntakeSpeedTop = 1;
  public static double intakeIntakeSpeedBtm = 1;

  public static double intakeEjectSpeedTop = 1;
  public static double intakeEjectSpeedBtm = 1;
}
