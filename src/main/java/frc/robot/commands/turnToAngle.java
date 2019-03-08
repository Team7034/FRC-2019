/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.TimedCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class turnToAngle extends TimedCommand {
  /**
   * Add your docs here.
   */
  private PIDController leftPID;
  private PIDController rightPID;
  private double target;

  public turnToAngle(double target_angle) {
    //super(Math.abs((Robot.m_driveTrain.getAngle() - target_angle) / 180)); //.5 seconds to turn 90 deg
    super(2);
    target = target_angle;
    requires(Robot.m_driveTrain);
    leftPID = new PIDController(.002,0,0.000,0, Robot.m_driveTrain.gyro, Robot.m_driveTrain.talonL);
		rightPID = new PIDController(.002,0,0.000,0, Robot.m_driveTrain.gyro, Robot.m_driveTrain.talonR);
    leftPID.setSetpoint(target);
    rightPID.setSetpoint(target);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    leftPID.enable();
    rightPID.enable();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    double setpoint = leftPID.getSetpoint();
    double setpoint2 = rightPID.getSetpoint();
    SmartDashboard.putNumber("leftPID", setpoint);
    SmartDashboard.putNumber("rightPID", setpoint2);
  }

  // Called once after timeout
  @Override
  protected void end() {
    leftPID.disable();
    rightPID.disable();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
