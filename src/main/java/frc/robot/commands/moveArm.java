/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;


public class moveArm extends Command {
  private int target; //target arm position
  private boolean finished = false; //has the arm reached its pos?
  public moveArm(int target) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.target = target;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //calculate where the arm needs to go based off of target and robot's current heading

    //targets 4, 5, 6 are for balls. arm angle does not tell difference between hatch and balls
    //if(target > 3) { target -= 3; }
    //0 is upright, does not need adj. if the arm needs to go to the opposite side, add 3 to convert
    //if(!Robot.arm_forward && target != 0) { target += 3; }

    //SmartDashboard.putNumber("target", target); //debugging
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    finished = Robot.m_arm.setArm(this.target);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return finished;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}

