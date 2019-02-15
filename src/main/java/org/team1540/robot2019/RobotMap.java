package org.team1540.robot2019;

import edu.wpi.first.wpilibj.SPI.Port;

public class RobotMap {

  // motors
  public static final int DRIVE_LEFT_A = 13;
  public static final int DRIVE_LEFT_B = 12;
  public static final int DRIVE_LEFT_C = 11;

  public static final int DRIVE_RIGHT_A = 1;
  public static final int DRIVE_RIGHT_B = 2;
  public static final int DRIVE_RIGHT_C = 3;

  public static final int ELEVATOR_L = 6;
  public static final int ELEVATOR_R = 7;

  public static final int INTAKE_WRIST = 8;

  public static final int INTAKE_TOP = 9;
  public static final int INTAKE_BTM = 10;

  public static final int CLIMBER_ARM_L = 5;
  public static final int CLIMBER_ARM_R = 4;

  // pneumatics

  public static final int CLIMBER_CYLINDER_1 = 2;
  public static final int CLIMBER_CYLINDER_2 = 3;

  public static final int HATCH_SLIDE = 1;
  public static final int HATCH_GRABBER = 0;

  // sensors and switches
  public static final int ELEVATOR_LIMIT_SENSOR = 1;

  public static final int WRIST_MID_SW = 2;
  public static final int WRIST_BTM_SW = 3;

  public static final int INTAKE_SENSOR = 0;

  public static final int PRESSURE_SENSOR = 3;


  // pdp ports
  public static final int PDP_DRIVE_LEFT_A = 15;
  public static final int PDP_DRIVE_LEFT_B = 14;
  public static final int PDP_DRIVE_LEFT_C = 13;
  public static final int PDP_DRIVE_RIGHT_A = 0;
  public static final int PDP_DRIVE_RIGHT_B = 2;
  public static final int PDP_DRIVE_RIGHT_C = 1;
  public static final int PDP_INTAKE_WRIST = 0;
  public static final int PDP_INTAKE_TOP = 0;
  public static final int PDP_INTAKE_BTM = 0;
  public static final int PDP_CLIMBER_ARM_L = 0;
  public static final int PDP_CLIMBER_ARM_R = 0;

  public static final Port NAVX = Port.kMXP;
}
