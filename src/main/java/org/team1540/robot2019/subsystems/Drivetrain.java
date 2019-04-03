package org.team1540.robot2019.subsystems;

import static org.team1540.robot2019.Hardware.DRIVE_POSITION_SLOT_IDX;
import static org.team1540.robot2019.Hardware.DRIVE_VELOCITY_SLOT_IDX;
import static org.team1540.robot2019.Hardware.driveLeftMotorA;
import static org.team1540.robot2019.Hardware.driveLeftMotorB;
import static org.team1540.robot2019.Hardware.driveLeftMotorC;
import static org.team1540.robot2019.Hardware.driveMotorMasters;
import static org.team1540.robot2019.Hardware.driveRightMotorA;
import static org.team1540.robot2019.Hardware.driveRightMotorB;
import static org.team1540.robot2019.Hardware.driveRightMotorC;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.IMotorController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.apache.log4j.Logger;
import org.jetbrains.annotations.NotNull;
import org.team1540.robot2019.Hardware;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.commands.drivetrain.DriveCommand;
import org.team1540.robot2019.datastructures.twod.Twist2D;
import org.team1540.rooster.drive.pipeline.DriveData;
import org.team1540.rooster.drive.pipeline.TankDriveData;
import org.team1540.rooster.functional.Output;
import org.team1540.rooster.wrappers.ChickenController;
import org.team1540.rooster.wrappers.ChickenTalon;

public class Drivetrain extends Subsystem {

    private static final Logger logger = Logger.getLogger(Drivetrain.class);

    boolean inFineDrive = false;

    private NetworkTable table = NetworkTableInstance.getDefault().getTable("drivetrain");
    private NetworkTableEntry leftPositionEntry = table.getEntry("leftPos");
    private NetworkTableEntry rightPositionEntry = table.getEntry("rightPos");

    private NetworkTableEntry leftVelocityEntry = table.getEntry("leftVel");
    private NetworkTableEntry rightVelocityEntry = table.getEntry("rightVel");

    private NetworkTableEntry leftThrottleEntry = table.getEntry("leftThrot");
    private NetworkTableEntry rightThrottleEntry = table.getEntry("rightThrot");

    private NetworkTableEntry leftCurrentAEntry = table.getEntry("leftCurrA");
    private NetworkTableEntry leftCurrentBEntry = table.getEntry("leftCurrB");
    private NetworkTableEntry leftCurrentCEntry = table.getEntry("leftCurrC");

    private NetworkTableEntry rightCurrentAEntry = table.getEntry("rightCurrA");
    private NetworkTableEntry rightCurrentBEntry = table.getEntry("rightCurrB");
    private NetworkTableEntry rightCurrentCEntry = table.getEntry("rightCurrC");

    private DriveCommand driveCommand;

    public DriveCommand getDriveCommand() {
        if (driveCommand == null) {
            driveCommand = new DriveCommand();
        }
        return driveCommand;
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(getDriveCommand());
    }

    public Output<TankDriveData> getPipelineOutput() {
        return getPipelineOutput(true);
    }

    public Output<TankDriveData> getPipelineOutput(boolean useClosedLoop) {
        return new Output<>() {
            public void accept(@NotNull TankDriveData tankDriveData) {
                processSide(tankDriveData.left, driveLeftMotorA);
                processSide(tankDriveData.right, driveRightMotorA);
            }

            private void processSide(DriveData data, IMotorController controller) {
                if (data.position.isPresent() && useClosedLoop) {
                    controller.selectProfileSlot(DRIVE_POSITION_SLOT_IDX, 0);
                    if (data.additionalFeedForward.isPresent()) {
                        controller.set(ControlMode.Position, data.position.getAsDouble(),
                            DemandType.ArbitraryFeedForward, data.additionalFeedForward.getAsDouble());
                    } else {
                        controller.set(ControlMode.Position, data.position.getAsDouble());
                    }
                } else if (data.velocity.isPresent() && useClosedLoop) {
                    controller.selectProfileSlot(DRIVE_VELOCITY_SLOT_IDX, 0);
                    if (data.additionalFeedForward.isPresent()) {
                        controller.set(ControlMode.Velocity, data.velocity.getAsDouble(),
                            DemandType.ArbitraryFeedForward, data.additionalFeedForward.getAsDouble());
                    } else {
                        controller.set(ControlMode.Velocity, data.velocity.getAsDouble());
                    }
                } else {
                    controller.set(ControlMode.PercentOutput, data.additionalFeedForward.orElse(0));
                }
            }
        };
    }

    public void splitFollowers() {
        for (ChickenController controller : Hardware.driveMotorAll) {
            controller.set(ControlMode.PercentOutput, 0);
        }
    }

    public void joinFollowers() {
        driveLeftMotorB.follow(driveLeftMotorA);
        driveLeftMotorC.follow(driveLeftMotorA);
        driveRightMotorB.follow(driveRightMotorA);
        driveRightMotorC.follow(driveRightMotorA);
    }

    private void configTalonsForPosition() {
        for (ChickenTalon t : driveMotorMasters) {
            t.selectProfileSlot(DRIVE_POSITION_SLOT_IDX);
        }
    }

    private void configTalonsForVelocity() {
        for (ChickenTalon t : driveMotorMasters) {
            t.selectProfileSlot(DRIVE_VELOCITY_SLOT_IDX);
        }
    }

    public void setPositionTicks(double left, double right) {
        setLeftPositionTicks(left);
        setRightPositionTicks(right);
    }

    public void setLeftPositionTicks(double position) {
        configTalonsForPosition();
        driveLeftMotorA.set(ControlMode.Position, position);
    }

    public void setRightPositionTicks(double position) {
        configTalonsForPosition();
        driveRightMotorA.set(ControlMode.Position, position);
    }

    public void setPositionTicks(double left, double right, double leftFeedForward,
        double rightFeedForward) {
        setLeftPositionTicks(left, leftFeedForward);
        setRightPositionTicks(right, rightFeedForward);
    }

    public void setLeftPositionTicks(double position, double feedForward) {
        configTalonsForPosition();
        driveLeftMotorA
            .set(ControlMode.Position, position, DemandType.ArbitraryFeedForward, feedForward);
    }

    public void setRightPositionTicks(double position, double feedForward) {
        configTalonsForPosition();
        driveRightMotorA
            .set(ControlMode.Position, position, DemandType.ArbitraryFeedForward, feedForward);
    }


    public void setVelocityTicks(double left, double right) {
        setLeftVelocityTicks(left);
        setRightVelocityTicks(right);
    }

    public void setLeftVelocityTicks(double velocity) {
        configTalonsForVelocity();
        driveLeftMotorA.set(ControlMode.Velocity, velocity);
    }

    public void setRightVelocityTicks(double velocity) {
        configTalonsForVelocity();
        driveRightMotorA.set(ControlMode.Velocity, velocity);
    }

    public void setVelocityTicks(double left, double right, double leftFeedForward,
        double rightFeedForward) {
        setLeftVelocityTicks(left, leftFeedForward);
        setRightVelocityTicks(right, rightFeedForward);
    }

    public void setLeftVelocityTicks(double velocity, double feedForward) {
        configTalonsForVelocity();
        driveLeftMotorA
            .set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedForward);
    }

    public void setRightVelocityTicks(double velocity, double feedForward) {
        configTalonsForVelocity();
        driveRightMotorA
            .set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedForward);
    }

    public void setLeftVelocityMetersPerSecond(double velocity) {
        setLeftVelocityTicks(velocity / 10 * Tuning.drivetrainTicksPerMeter);
    }

    public void setRightVelocityMetersPerSecond(double velocity) {
        setRightVelocityTicks(velocity / 10 * Tuning.drivetrainTicksPerMeter);
    }

    public void setPercent(double left, double right) {
        setLeftPercent(left);
        setRightPercent(right);
    }

    public void setLeftPercent(double percent) {
        driveLeftMotorA.set(ControlMode.PercentOutput, percent);
    }

    public void setRightPercent(double percent) {
        driveRightMotorA.set(ControlMode.PercentOutput, percent);
    }

    public void setBrake(boolean brake) {
        for (ChickenController talon : driveMotorMasters) {
            talon.setBrake(brake);
        }
    }

    public void setTwist(Twist2D cmdVel) {
        double leftSetpoint = cmdVel.getX() - cmdVel.getOmega() * Tuning.drivetrainRadiusMeters;
        double rightSetpoint = cmdVel.getX() + cmdVel.getOmega() * Tuning.drivetrainRadiusMeters;
        setLeftVelocityMetersPerSecond(leftSetpoint);
        setRightVelocityMetersPerSecond(rightSetpoint);
//        cmdVel.putToNetworkTable("Debug/DriveTrain/CmdVel");
    }

    public void setPercentTwist(Twist2D cmdVel) {
        double leftSetpoint = cmdVel.getX() - cmdVel.getOmega();
        double rightSetpoint = cmdVel.getX() + cmdVel.getOmega();
        setLeftPercent(leftSetpoint);
        setRightPercent(rightSetpoint);
//        cmdVel.putToNetworkTable("Debug/DriveTrain/CmdVelPercent");
    }

    public double getLeftPositionTicks() {
        return driveLeftMotorA.getSelectedSensorPosition();
    }

    public double getRightPositionTicks() {
        return driveRightMotorA.getSelectedSensorPosition();
    }

    public double getLeftPositionMeters() {
        return getLeftPositionTicks() / Tuning.drivetrainTicksPerMeter;
    }

    public double getRightPositionMeters() {
        return getRightPositionTicks() / Tuning.drivetrainTicksPerMeter;
    }

    public double getLeftVelocityTicks() {
        return driveLeftMotorA.getSelectedSensorVelocity();
    }

    public double getRightVelocityTicks() {
        return driveRightMotorA.getSelectedSensorVelocity();
    }

    public double getLeftVelocityMetersPerSecond() {
        return getLeftVelocityTicks() * 10 / Tuning.drivetrainTicksPerMeter;
    }

    public double getRightVelocityMetersPerSecond() {
        return getRightVelocityTicks() * 10 / Tuning.drivetrainTicksPerMeter;
    }

    public Twist2D getTwist() {
        double xvel = (getLeftVelocityMetersPerSecond() + getRightVelocityMetersPerSecond()) / 2;
        double thetavel =
            (getLeftVelocityMetersPerSecond() - getRightVelocityMetersPerSecond()) / (Tuning.drivetrainRadiusMeters)
                / 2;
        return new Twist2D(xvel, 0, thetavel);
    }

    public double getLeftPercent() {
        return driveLeftMotorA.getMotorOutputPercent();
    }

    public double getRightPercent() {
        return driveRightMotorA.getMotorOutputPercent();
    }

    public void zeroEncoders() {
        Hardware.driveLeftMotorA.setSelectedSensorPosition(0);
        Hardware.driveRightMotorA.setSelectedSensorPosition(0);
    }

    public void stop() {
        setLeftVelocityTicks(0);
        setRightVelocityTicks(0);
    }

    public NetworkTableEntry getLeftPositionEntry() {
        return leftPositionEntry;
    }

    public NetworkTableEntry getRightPositionEntry() {
        return rightPositionEntry;
    }

    public NetworkTableEntry getLeftVelocityEntry() {
        return leftVelocityEntry;
    }

    public NetworkTableEntry getRightVelocityEntry() {
        return rightVelocityEntry;
    }

    public NetworkTableEntry getLeftThrottleEntry() {
        return leftThrottleEntry;
    }

    public NetworkTableEntry getRightThrottleEntry() {
        return rightThrottleEntry;
    }

    public NetworkTableEntry getLeftCurrentAEntry() {
        return leftCurrentAEntry;
    }

    public NetworkTableEntry getLeftCurrentBEntry() {
        return leftCurrentBEntry;
    }

    public NetworkTableEntry getLeftCurrentCEntry() {
        return leftCurrentCEntry;
    }

    public NetworkTableEntry getRightCurrentAEntry() {
        return rightCurrentAEntry;
    }

    public NetworkTableEntry getRightCurrentBEntry() {
        return rightCurrentBEntry;
    }

    public NetworkTableEntry getRightCurrentCEntry() {
        return rightCurrentCEntry;
    }

    @Override
    public void periodic() {
        if (Robot.debugMode) {
            leftPositionEntry.setNumber(getLeftPositionTicks());
            rightPositionEntry.setNumber(getRightPositionTicks());

            leftVelocityEntry.setNumber(getLeftVelocityTicks());
            rightVelocityEntry.setNumber(getRightVelocityTicks());

            leftThrottleEntry.setNumber(getLeftPercent());
            rightThrottleEntry.setNumber(getRightPercent());

            leftCurrentAEntry.setNumber(Hardware.getDriveLeftACurrent());
            leftCurrentBEntry.setNumber(Hardware.getDriveLeftBCurrent());
            leftCurrentCEntry.setNumber(Hardware.getDriveLeftCCurrent());

            rightCurrentAEntry.setNumber(Hardware.getDriveRightACurrent());
            rightCurrentBEntry.setNumber(Hardware.getDriveRightBCurrent());
            rightCurrentCEntry.setNumber(Hardware.getDriveRightCCurrent());
        }
    }
}

