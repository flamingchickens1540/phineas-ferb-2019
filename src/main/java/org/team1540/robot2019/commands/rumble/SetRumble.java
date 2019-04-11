package org.team1540.robot2019.commands.rumble;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Command;

public class SetRumble extends Command { // TODO: Rooster

    private GenericHID xboxController;
    private double value;

    public SetRumble(XboxController xboxController, double value) {
        this.xboxController = xboxController;
        this.value = value;
    }

    @Override
    protected void initialize() {
        xboxController.setRumble(RumbleType.kLeftRumble, value);
        xboxController.setRumble(RumbleType.kRightRumble, value);
    }

    @Override
    protected boolean isFinished() {
        return true;
    }
}
