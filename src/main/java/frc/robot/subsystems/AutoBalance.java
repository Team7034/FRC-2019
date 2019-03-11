/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class AutoBalance extends Command {
  public AutoBalance() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.m_arm);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.m_arm.arm_forward = true;
    Robot.m_arm.controller.setOutputRange(-.1, .1);
    Robot.m_arm.controller.setP(0.1);
    Robot.m_arm.controller.setD(.13);
    Robot.m_arm.controller.setF(0.035);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double target = 0;
    double roll = Robot.m_driveTrain.gyro.getRoll();
    /*if(roll < 0){
      target = .4* roll;
    } else if(roll > 0 && roll < 1.5){
      target = -.25;
    } else if(roll > 1.5){
      target = .2* roll;
    }*/
    
    /*if(target > 0){
      target = 0;
    }else if(target < -1){
      target = -1;
    }*/

    target = .4*roll;
    Robot.m_arm.controller.setSetpoint(target);
    
    /*
    SmartDashboard.putNumber("AutoBal Tar", target);
    SmartDashboard.putNumber("Arm Enc", m_arm.get_arm_pos());*/
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_arm.controller.setOutputRange(-.25, .25);
    Robot.m_arm.controller.setD(.16);
    Robot.m_arm.controller.setSetpoint(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.m_arm.controller.setOutputRange(-.25, .25);
    Robot.m_arm.controller.setD(.16);
    Robot.m_arm.controller.setSetpoint(0);
  }
}
