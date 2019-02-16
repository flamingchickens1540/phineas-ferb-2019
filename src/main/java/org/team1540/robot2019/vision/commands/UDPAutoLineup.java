package org.team1540.robot2019.vision.commands;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Command;
import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.datastructures.threed.Transform3D;
import org.team1540.robot2019.datastructures.twod.Twist2D;
import org.team1540.robot2019.networking.UDPOdometryGoalSender;
import org.team1540.robot2019.networking.UDPTwistReceiver;
import org.team1540.robot2019.subsystems.Drivetrain;
import org.team1540.robot2019.utils.LimelightLocalization;
import org.team1540.robot2019.utils.TankDriveOdometryRunnable;
import org.team1540.robot2019.utils.TankDriveTwist2DInput;
import org.team1540.robot2019.utils.TrigUtils;
import org.team1540.rooster.drive.pipeline.FeedForwardProcessor;
import org.team1540.rooster.drive.pipeline.UnitScaler;
import org.team1540.rooster.functional.Executable;

public class UDPAutoLineup extends Command {

    private final Drivetrain dt;
    private final UDPOdometryGoalSender sender;
  private final UDPTwistReceiver receiver;
  private final LimelightLocalization limeLoc;
  private final TankDriveOdometryRunnable driveOdometry;
  private final Transform3D lastOdomToLimelight;
  private final AHRS navx;

  Transform3D goal;
  private Executable pipeline;
  private TankDriveTwist2DInput twist2DInput;

  public UDPAutoLineup(Drivetrain dt,UDPOdometryGoalSender sender, UDPTwistReceiver receiver, LimelightLocalization limeLoc, TankDriveOdometryRunnable driveOdometry, Transform3D lastOdomToLimelight, AHRS navx) {
      this.dt = dt;
      this.sender = sender;
    this.receiver = receiver;
    this.limeLoc = limeLoc;
    this.driveOdometry = driveOdometry;
    this.lastOdomToLimelight = lastOdomToLimelight;
    this.navx = navx;
    requires(dt);
    twist2DInput = new TankDriveTwist2DInput(Tuning.drivetrainRadius);
    pipeline = twist2DInput
        .then(new FeedForwardProcessor(0, 0,0))
//        .then(new FeedForwardProcessor(0.27667, 0.054083,0.08694))
        // .then((Processor<TankDriveData, TankDriveData>) tankDriveData -> new TankDriveData(tankDriveData.left, tankDriveData.right))
        .then(new UnitScaler(Tuning.drivetrainTicksPerMeter, 10))
        .then(dt.getPipelineOutput());
  }

  @Override
  protected void initialize() {
//    dt.reset();
    dt.configTalonsForVelocity();


    NetworkTable tebConfigTable = NetworkTableInstance.getDefault().getTable("TEBPlanner/Config");
    tebConfigTable.getEntry("TEBReset").setBoolean(true);
      tebConfigTable.getEntry("MaxVelX").setNumber(1.5);
      tebConfigTable.getEntry("MaxVelXBackwards").setNumber(1.4);
      tebConfigTable.getEntry("AccLimX").setNumber(1.0);
      tebConfigTable.getEntry("MaxVelTheta").setNumber(5.0);
    tebConfigTable.getEntry("AccLimTheta").setNumber(12.0);
    if (limeLoc.attemptUpdatePose()) { // TODO: Make this distance tunable
      computeAndUpdateGoal();
    } else {
//      if (limeLoc.millisSinceLastAcquired() < 0) {
//        updateGoal(lastOdomToLimelight);
//      } else {
  //      Robot.leds.set(ColorPattern.RED);
        cancel();
//      }
    }
  }

  private void computeAndUpdateGoal() {
    updateGoal(computeGoal());
  }

  private Transform3D computeGoal() {
    return driveOdometry.getOdomToBaseLink()
        .add(limeLoc.getBaseLinkToVisionTarget())
        .add(new Transform3D(new Vector3D(-0.65, -0.025, 0), Rotation.IDENTITY));
  }

  private void updateGoal(Transform3D newGoal) {
    this.goal = newGoal;
    sender.setGoal(goal.toTransform2D());
    System.out.println("Goal updated");

    Transform3D via_point = goal.add(new Transform3D(-0.7, 0, 0));
    sender.setViaPoint(via_point.toTransform2D().getPositionVector());
  }

  @Override
  protected void execute() {
      if (limeLoc.attemptUpdatePose() && (getDistanceError() > 0.05)) { // TODO: Make this distance tunable
      computeAndUpdateGoal();
    }

    // Send velocity command
    Twist2D cmdVel = receiver.getCmdVel();
    twist2DInput.setTwist(cmdVel);
    pipeline.execute();
  }

  @Override
  protected boolean isFinished() {
    if (goal == null) {
      return true;
    }
    if (getDistanceError() < 0.02 && Math.abs(getAngleError()) < Math.toRadians(3)) {
      dt.stop();
//      Robot.leds.set(ColorPattern.LIME);
      return true;
    }
    return false;
  }

  private double getDistanceError() {
    Vector3D odomPosition = driveOdometry.getOdomToBaseLink().getPosition(); // TODO: This should use javaTF
    return goal.toTransform2D().getPositionVector().distance(new Vector2D(odomPosition.getX(), odomPosition.getY()));
  }

  private double getAngleError() {
    return TrigUtils.SignedAngleDifference(goal.toTransform2D().getTheta(), Math.toRadians(-navx.getYaw()));
  }
}
