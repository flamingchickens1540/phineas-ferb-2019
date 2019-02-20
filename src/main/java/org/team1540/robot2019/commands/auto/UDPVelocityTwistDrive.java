package org.team1540.robot2019.commands.auto;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.team1540.robot2019.Hardware;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.datastructures.twod.Transform2D;
import org.team1540.robot2019.datastructures.twod.Twist2D;
import org.team1540.robot2019.datastructures.utils.TrigUtils;
import org.team1540.robot2019.utils.TankDriveTwist2DInput;
import org.team1540.rooster.drive.pipeline.CTREOutput;
import org.team1540.rooster.drive.pipeline.FeedForwardProcessor;
import org.team1540.rooster.drive.pipeline.UnitScaler;
import org.team1540.rooster.functional.Executable;

// TODO: Use proper logging class
public class UDPVelocityTwistDrive extends Command {

    Transform2D goal;
    private Executable pipeline;
    private TankDriveTwist2DInput twist2DInput;

    boolean freeGoalVel;
    boolean checkEnd;

    public UDPVelocityTwistDrive(Transform2D goal, boolean freeGoalVel) {
        SmartDashboard.setDefaultNumber("test-goal/position/x", 2);
        SmartDashboard.setDefaultNumber("test-goal/position/y", 0);
        SmartDashboard.setDefaultNumber("test-goal/orientation/z", 0);

        this.goal = goal;
        this.freeGoalVel = freeGoalVel;

        this.checkEnd = true;
        requires(Robot.drivetrain);
    }

    public UDPVelocityTwistDrive() {
        this.checkEnd = true;
        this.freeGoalVel = false;
        requires(Robot.drivetrain);
    }

    @Override
    protected void initialize() {
        twist2DInput = new TankDriveTwist2DInput(Tuning.drivetrainRadiusMeters);
        pipeline = twist2DInput
            .then(new FeedForwardProcessor(Tuning.driveKV, Tuning.driveVIntercept, 0))
            .then(new UnitScaler(Tuning.drivetrainTicksPerMeter, 10))
            .then(new CTREOutput(Hardware.driveLeftMotorA, Hardware.driveRightMotorA, true));

        NetworkTable tebConfigTable = NetworkTableInstance.getDefault().getTable("TEBPlanner/Config");
        tebConfigTable.getEntry("TEBReset").setBoolean(true);
        if (tebConfigTable.getEntry("ResetTuningVals").getBoolean(true)) {
            tebConfigTable.getEntry("MaxVelX").setNumber(1.5);
            tebConfigTable.getEntry("MaxVelXBackwards").setNumber(1.4);
            tebConfigTable.getEntry("AccLimX").setNumber(1.5);
            tebConfigTable.getEntry("MaxVelTheta").setNumber(5.0);
            tebConfigTable.getEntry("AccLimTheta").setNumber(15.0);
        }
        updateGoal();
    }

    private void updateGoal() {
        double xGoal = SmartDashboard.getNumber("test-goal/position/x", 1);
        double yGoal = SmartDashboard.getNumber("test-goal/position/y", 0);
        double angleGoal = SmartDashboard.getNumber("test-goal/orientation/z", 0);
        System.out.println("Updated goal!");

        goal = Robot.odometry.getOdomToBaseLink().toTransform2D().add(new Transform2D(xGoal, yGoal, angleGoal));
//        .add(Robot.limelightLocalization.getLastBaseLinkToVisionTarget())
//        .add(new Transform3D(new Vector3D(-0.65, 0, 0), Rotation.IDENTITY));

//    Robot.odometry.reset();
        Robot.tebPlanner.setGoal(goal);
        Robot.tebPlanner.setViaPoint(goal.getPositionVector());
//    Robot.udpSender.setViaPoint(new Vector2D(1, -1));
    }

    @Override
    protected void execute() {
        Twist2D cmdVel = Robot.tebPlanner.getCmdVel();
        double leftSetpoint = (cmdVel.getX() - cmdVel.getOmega() * Tuning.drivetrainRadiusMeters) * Tuning.drivetrainTicksPerMeter / 10;
        double rightSetpoint = (cmdVel.getX() + cmdVel.getOmega() * Tuning.drivetrainRadiusMeters) * Tuning.drivetrainTicksPerMeter / 10;
        Robot.drivetrain.setLeftVelocityTicks(leftSetpoint);
        Robot.drivetrain.setRightVelocityTicks(rightSetpoint);
        SmartDashboard.putNumber("debug-setpoint-left", leftSetpoint * 10 / Tuning.drivetrainTicksPerMeter);
        SmartDashboard.putNumber("debug-setpoint-right", rightSetpoint * 10 / Tuning.drivetrainTicksPerMeter);
        SmartDashboard.putNumber("debug-velocity-left", Robot.drivetrain.getLeftVelocityMetersPerSecond());
        SmartDashboard.putNumber("debug-velocity-right", Robot.drivetrain.getRightVelocityMetersPerSecond());
//    twist2DInput.setTwist(cmdVel);
//    pipeline.execute();
    }

    @Override
    protected boolean isFinished() {
        if (!this.checkEnd) {
            return false;
        }
        // isFinished Checking
        Vector3D odomPosition = Robot.odometry.getOdomToBaseLink().getPosition(); // TODO: This should use javaTF
        double xError = goal.getX() - odomPosition.getX();
        double yError = goal.getY() - odomPosition.getY();
        double angleError = TrigUtils
            .signedAngleError(Hardware.navx.getYawRadians(), goal.getTheta()); // TODO: If this is the proper way to calculate signed angle, this should be moved to the TrigUtils class

        boolean finished = Math.abs(xError) < 0.05 && // TODO: Make this a static function
            Math.abs(yError) < 0.05 &&
            Math.abs(angleError) < Math.toRadians(3);
        if (finished) {
            System.out.println("Close to goalAvg: " + goal.getX() + " " + goal.getY());
            Robot.drivetrain.stop();
        }
        return finished;
    }
}