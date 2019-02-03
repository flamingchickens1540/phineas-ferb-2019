package org.team1540.robot2019;

import edu.wpi.first.wpilibj.SPI.Port;

public class RobotMap {

  // motors
  public static final int DRIVE_LEFT_A = 11;
  public static final int DRIVE_LEFT_B = 12;
  public static final int DRIVE_LEFT_C = 13;

  public static final int DRIVE_RIGHT_A = 1;
  public static final int DRIVE_RIGHT_B = 2;
  public static final int DRIVE_RIGHT_C = 3;

  public static final int ELEVATOR_L = 6;
  public static final int ELEVATOR_R = 7;

  public static final int INTAKE_WRIST = 9;

  public static final int INTAKE_TOP = 8;
  public static final int INTAKE_BTM = 10;

  public static final int CLIMBER_ARM_L = 4;
  public static final int CLIMBER_ARM_R = 5;

  // pneumatics

  public static final int CLIMBER_CYLINDER_1 = 6;
  public static final int CLIMBER_CYLINDER_2 = 7;

  public static final int HATCH_SLIDE = 2;

  // sensors and switches
  public static final int ELEVATOR_LIMIT_SENSOR = 1;

  public static final int WRIST_MID_SW = 2;
  public static final int WRIST_BTM_SW = 3;

  public static final int INTAKE_SENSOR = 0;

  public static final int PRESSURE_SENSOR = 3;

  public static final Port NAVX = Port.kMXP;
}
