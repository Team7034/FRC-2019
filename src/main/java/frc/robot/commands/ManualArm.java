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

public class ManualArm extends Command {
  double target;
  int ele_tar;
  public ManualArm() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_arm);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(Robot.m_arm.run_manual_arm){
      Robot.m_arm.manual_arm(.3*Robot.m_oi.getArm());
    }
    Robot.m_arm.manual_ele(-Robot.m_oi.getElevator());
    SmartDashboard.putNumber("ArmEnc", Robot.m_arm.get_arm_pos());
    //SmartDashboard.putNumber("ArmTar", Robot.m_arm.arm_target);
    //SmartDashboard.putNumber("ArmTar", Robot.m_oi.getArm());
    SmartDashboard.putNumber("EleEnc", Robot.m_arm.get_ele_pos());
    }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
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
