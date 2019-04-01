/*----------------------------------------------------------------------------*/
/* Copyright (c) 2008-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team1540.robot2019.commands.auto;

import edu.wpi.first.wpilibj.command.Command;
import java.util.function.BooleanSupplier;

public class WaitUntilCommand extends Command {

    private final BooleanSupplier isFinished;
    private final Runnable end;

    public WaitUntilCommand(BooleanSupplier isFinished, Runnable end) {
        this.isFinished = isFinished;
        this.end = end;
    }

    public WaitUntilCommand(BooleanSupplier isFinished) {
        this(isFinished, null);
    }

    @Override
    protected boolean isFinished() {
        return isFinished.getAsBoolean();
    }

    @Override
    protected void end() {
        end.run();
    }
}
