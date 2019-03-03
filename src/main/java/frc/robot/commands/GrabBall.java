/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.arm;

public class GrabBall extends CommandGroup {
  /**
   * Moves arm to intake position until ball is grabbed, moves arm to center
   */
  public GrabBall() {
    //move arm into intake position
    addSequential(new AutomaticArm(arm.state.get("intake")));
    //open intake, turn on intake sucker
    addParallel(new Grab(false));
    addSequential(new RunIntake(Robot.m_claw.target_power));
    //wait until ball is grabbed
    addSequential(new AutoGrab());
    //move arm to center when grabbed
    addSequential(new AutomaticArm(arm.state.get("rest")));
  }
}
