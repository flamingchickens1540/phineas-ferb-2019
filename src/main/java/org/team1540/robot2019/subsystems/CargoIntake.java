package org.team1540.robot2019.subsystems;

import static org.team1540.robot2019.Hardware.intakeBtm;
import static org.team1540.robot2019.Hardware.intakeSensor;
import static org.team1540.robot2019.Hardware.intakeTop;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.team1540.robot2019.Hardware;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;

public class CargoIntake extends Subsystem {

    private NetworkTable table = NetworkTableInstance.getDefault().getTable("cargoIntake");
    private NetworkTableEntry hasBallEntry = table.getEntry("hasBall");
    private NetworkTableEntry topThrottleEntry = table.getEntry("topThrot");
    private NetworkTableEntry topCurrentEntry = table.getEntry("topCurr");
    private NetworkTableEntry btmThrottleEntry = table.getEntry("btmThrot");
    private NetworkTableEntry btmCurrentEntry = table.getEntry("btmCurr");

    @Override
    protected void initDefaultCommand() {

    }

    public void startIntaking() {
        intakeTop.set(ControlMode.PercentOutput, -Tuning.intakeIntakeSpeedTop);
        intakeBtm.set(ControlMode.PercentOutput, -Tuning.intakeIntakeSpeedBtm);
    }

    public void startEjecting() {
        intakeTop.set(ControlMode.PercentOutput, Tuning.intakeEjectSpeedTop);
        intakeBtm.set(ControlMode.PercentOutput, Tuning.intakeEjectSpeedBtm);
    }

    public void stop() {
        intakeTop.set(ControlMode.PercentOutput, 0);
        intakeBtm.set(ControlMode.PercentOutput, 0);
    }

    public boolean hasBall() {
        return intakeSensor.get();
    }

    @Override
    public void periodic() {
        if (Robot.debugMode) {
            hasBallEntry.forceSetBoolean(hasBall());
            topThrottleEntry.forceSetNumber(intakeTop.getMotorOutputPercent());
            topCurrentEntry.forceSetNumber(Hardware.getIntakeTopCurrent());
            btmThrottleEntry.forceSetNumber(intakeBtm.getMotorOutputPercent());
            btmCurrentEntry.forceSetNumber(Hardware.getIntakeBtmCurrent());
        }
    }
}
