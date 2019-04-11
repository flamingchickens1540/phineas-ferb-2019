package org.team1540.robot2019.subsystems;

import static org.team1540.robot2019.Hardware.cargoIntakeSensor;
import static org.team1540.robot2019.Hardware.cargoRollerBottom;
import static org.team1540.robot2019.Hardware.cargoRollerTop;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.team1540.robot2019.Hardware;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;

public class CargoMech extends Subsystem {

    private NetworkTable table = NetworkTableInstance.getDefault().getTable("cargoMech");
    private NetworkTableEntry hasBallEntry = table.getEntry("hasBall");
    private NetworkTableEntry topThrottleEntry = table.getEntry("topThrot");
    private NetworkTableEntry topCurrentEntry = table.getEntry("topCurr");
    private NetworkTableEntry btmThrottleEntry = table.getEntry("btmThrot");
    private NetworkTableEntry btmCurrentEntry = table.getEntry("btmCurr");

    @Override
    protected void initDefaultCommand() {

    }

    public void startIntaking() {
        cargoRollerTop.set(ControlMode.PercentOutput, Tuning.cargoIntakeSpeedTop);
        cargoRollerBottom.set(ControlMode.PercentOutput, -Tuning.cargoIntakeSpeedBtm);
    }

    public void startEjecting() {
        cargoRollerTop.set(ControlMode.PercentOutput, -Tuning.cargoEjectSpeedTop);
        cargoRollerBottom.set(ControlMode.PercentOutput, Tuning.cargoEjectSpeedBtm);
    }

    public void stop() {
        cargoRollerTop.set(ControlMode.PercentOutput, 0);
        cargoRollerBottom.set(ControlMode.PercentOutput, 0);
    }

    public boolean hasBall() {
        return cargoIntakeSensor.get();
    }

    @Override
    public void periodic() {
        if (Robot.debugMode) {
            hasBallEntry.forceSetBoolean(hasBall());
            topThrottleEntry.forceSetNumber(cargoRollerTop.getMotorOutputPercent());
            topCurrentEntry.forceSetNumber(Hardware.getCargoMechTopCurrent());
            btmThrottleEntry.forceSetNumber(cargoRollerBottom.getMotorOutputPercent());
            btmCurrentEntry.forceSetNumber(Hardware.getCargoMechBtmCurrent());
        }
    }
}
