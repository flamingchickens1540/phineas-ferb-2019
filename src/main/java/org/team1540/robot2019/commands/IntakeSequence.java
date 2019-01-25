package org.team1540.robot2019.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.team1540.robot2019.commands.wrist.LowerWrist;
import org.team1540.robot2019.commands.wrist.RaiseWrist;
import org.team1540.robot2019.commands.intake.IntakeAuto;

public class IntakeSequence extends CommandGroup {

  public IntakeSequence() {
    addSequential(new LowerWrist());
    addSequential(new IntakeAuto());
    addSequential(new RaiseWrist());
  }
}
