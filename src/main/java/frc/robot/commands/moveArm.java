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


public class moveArm extends Command {
  public double target;
  public moveArm() {
    requires(Robot.m_arm);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    target = Robot.m_arm.getPos();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    target += Robot.m_oi.getRightX()/5;

    Robot.m_arm.setTarget(target);
    SmartDashboard.putNumber("ArmTarget", target);
    SmartDashboard.putNumber("ArmPower", Robot.m_arm.neo.getAppliedOutput());
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
