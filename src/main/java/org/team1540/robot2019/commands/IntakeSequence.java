package org.team1540.robot2019.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.team1540.robot2019.commands.arm.LowerArm;
import org.team1540.robot2019.commands.arm.RaiseArm;
import org.team1540.robot2019.commands.intake.IntakeAuto;

public class IntakeSequence extends CommandGroup {

  public IntakeSequence() {
    addSequential(new LowerArm());
    addSequential(new IntakeAuto());
    addSequential(new RaiseArm());
  }
}
