/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
/*
package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class moveLift extends Command {
  private int target;
  public moveLift() {
    requires(Robot.m_lift);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    target = Robot.m_lift.getPos();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    target += Robot.m_oi.getLeftY()*15000;
    Robot.m_lift.setTarget(target);
    SmartDashboard.putNumber("LiftPos", Robot.m_lift.getPos());
    SmartDashboard.putNumber("LiftPower", Robot.m_lift.talon.getMotorOutputPercent());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
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
*/
