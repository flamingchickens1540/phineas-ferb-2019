package org.team1540.robot2019.commands.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.apache.log4j.Logger;
import org.team1540.robot2019.commands.elevator.ElevatorSelfTest;
import org.team1540.robot2019.commands.hatch.HatchSelfTest;
import org.team1540.robot2019.commands.intake.IntakeSelfTest;
import org.team1540.robot2019.commands.wrist.WristSelfTest;
import org.team1540.rooster.util.SimpleCommand;

public class SelfTest extends CommandGroup {

  public static final Logger logger = Logger.getLogger(SelfTest.class);

  public SelfTest() {
    addSequential(new SimpleCommand("Print status", () -> logger.info("Starting robot self-test")));
    addSequential(new HatchSelfTest());
    addSequential(new IntakeSelfTest());
    addSequential(new WristSelfTest());
    addSequential(new ElevatorSelfTest());
    addSequential(new SimpleCommand("Print status", () -> logger.info("Robot self-test complete")));
  }
}