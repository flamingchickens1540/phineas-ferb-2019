package org.team1540.robot2019.tuners;

import edu.wpi.first.wpilibj.TimedRobot;

public class AutoAlignTuningRobot extends TimedRobot {
//
//  private static Joystick joystick = new Joystick(0);
//
////  public static RevBlinken leds = new RevBlinken(9);
//
//
//  static JoystickButton autoAlignButton = new JoystickButton(joystick, RB);
//  static JoystickButton autoAlignCancelButton = new JoystickButton(joystick, LB);
//
//  static Command alignCommand = null;
//
//
//  @Override
//  public void robotInit() {
////    AdjustableManager.getInstance().add(new Tuning());
//    PowerManager.getInstance().interrupt();
////    PreferenceManager.getInstance().add(new Tuning());
//      Scheduler.getInstance().run();
//      Hardware.initDrive();
//      OI.initJoysticks();
//
//    SmartDashboard.putNumber("angleMultiplier", 1);
//
//    drivetrain = new Drivetrain();
//
//    odometry = new TankDriveOdometryRunnable(
//        drivetrain::getLeftPositionMeters,
//        drivetrain::getRightPositionMeters,
//        Robot.navx::getYawRadians
//    );
//
//    udpReceiver = new UDPTwistReceiver(5801);
//
//    udpSender = new UDPOdometryGoalSender(Tuning.isComp ? "10.15.40.202" : "10.15.40.201", 5800);
//
//    Robot.limelightLocalization = new LimelightLocalization("limelight-a");
//
//    StateChangeDetector limelightStateDetector = new StateChangeDetector(false);
//
//    new Notifier(() -> {
//      odometry.run();
//      Robot.odomToBaseLink = odometry.getOdomToBaseLink();
//      udpSender.setOdometry(new Odometry(Robot.odomToBaseLink, drivetrain.getTwist()));
//      Robot.odomToBaseLink.toTransform2D().putToNetworkTable("Odometry/Debug/WheelOdometry");
//      boolean targetFound = Robot.limelightLocalization.attemptUpdatePose();
//      if (targetFound) {
//        Robot.limelightLocalization.getLastBaseLinkToVisionTarget().toTransform2D().putToNetworkTable("LimelightLocalization/Debug/BaseLinkToVisionTarget");
//        Transform3D goal = odometry.getOdomToBaseLink()
//            .add(Robot.limelightLocalization.getLastBaseLinkToVisionTarget())
//            .add(new Transform3D(new Vector3D(-0.65, 0, 0), Rotation.IDENTITY));
//
//          Robot.lastOdomToLimelightGoal = goal;
//        goal.toTransform2D().putToNetworkTable("LimelightLocalization/Debug/BaseLinkToGoal");
//      }
//      if (alignCommand == null || !alignCommand.isRunning()) {
//        if (targetFound) {
//          OI.driver.setRumble(RumbleType.kLeftRumble, 1);
//        } else {
//          OI.driver.setRumble(RumbleType.kLeftRumble, 0);
//        }
//      }
//      try {
//        udpSender.sendIt();
//      } catch (IOException e) {
//        DriverStation.reportWarning("Unable to send Odometry packet!", false);
//      }
//    }).startPeriodic(0.011);
//
//    // Testing code
//    Command testTEB = new SimpleCommand("Test TEB", () -> {
//      new UDPVelocityTwistDrive().start();
//    });
//    SmartDashboard.putData(testTEB);
//
//    SmartDashboard.putNumber("test-goal/position/x", 2);
//    SmartDashboard.putNumber("test-goal/position/y", 0);
//    SmartDashboard.putNumber("test-goal/orientation/z", 0);
//
//      // Testing code
//      Command resetWheelOdom = new SimpleCommand("Update PID Values", () -> {
//          drivetrain.updatePIDValues();
//        System.out.println(Tuning.driveVelocityP);
//      });
//      resetWheelOdom.setRunWhenDisabled(true);
//      SmartDashboard.putData(resetWheelOdom);
//
//    autoAlignButton.whenPressed(new SimpleCommand("Start Lineup", () -> {
////        alignCommand = new UDPAutoLineup(drivetrain, udpSender, udpReceiver, Robot.limelightLocalization, odometry, Robot.lastOdomToLimelightGoal, Robot.navx);
////        alignCommand = new UDPVelocityTwistDrive();
//      alignCommand.start();
//    }));
//    autoAlignCancelButton.whenPressed(new SimpleCommand("Cancel Lineup", () -> {
//      if (alignCommand != null) {
//        alignCommand.cancel();
//      }
//    }));
//
//    NetworkTable tebConfigTable = NetworkTableInstance.getDefault().getTable("TEBPlanner/Config");
//    tebConfigTable.getEntry("ResetTuningVals").setBoolean(true);
//  }
//
//  @Override
//  public void robotPeriodic() {
//
//    Scheduler.getInstance().run();
//
//      NetworkTableInstance nt = NetworkTableInstance.getDefault();
//    nt.getTable("Drivetrain/Debug/Position").getEntry("Left").setNumber(drivetrain.getLeftPositionTPU());
//    nt.getTable("Drivetrain/Debug/Position").getEntry("Right").setNumber(drivetrain.getRightPositionTPU());
//  }
//
//  @Override
//  public void disabledInit() {
//    drivetrain.setBrake(false);
//  }
//
//  @Override
//  public void teleopInit() {
//  }
}
