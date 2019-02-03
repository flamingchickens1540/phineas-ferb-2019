package org.team1540.robot2019.subsystems;

import static org.team1540.robot2019.Hardware.hatchSlide;

import edu.wpi.first.wpilibj.command.Subsystem;

public class HatchMech extends Subsystem {

  public void getHatch() {
    slideOut();
    attatch();
  }

  public void placeHatch() {
    release();
    slideIn();
  }

  public void slideOut() {
    hatchSlide.set(true);
  }

  public void slideIn() {
    hatchSlide.set(false);
  }

  public void attatch() {
  }

  public void release() {
  }

  @Override
  protected void initDefaultCommand() {
  }

}
