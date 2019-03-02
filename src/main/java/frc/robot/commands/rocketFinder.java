/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class rocketFinder extends Command {

  public double theta_bryce; //degrees and backwards (left = pos, right = neg)
  public double theta_ours = 9999; //degrees (left = neg, right = pos)
  public double current_angle; //degrees
  public double distanceFromRocket; //inches
  public double heightFromRocket; //inches
  public double lengthFromRocket; //inches
  
  public rocketFinder(double bryce_theta, double distance) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);

    requires(Robot.m_driveTrain);

    theta_bryce = bryce_theta;
    distanceFromRocket = distance;

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    current_angle = Robot.m_driveTrain.getAngle();
    if (theta_bryce != 0 && distanceFromRocket != 0)
    {
    theta_ours = 90 + theta_bryce - current_angle;
    }
    else
    theta_ours = 9999;
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double encoderPositionAtStart = (Robot.m_driveTrain.talonR.getSelectedSensorPosition() / 1399.539244); //in inches now
    if (theta_ours != 9999)
    {
      turnToAngle(theta_ours);
    }
   
    
  }

  protected void turnToAngle(double theta)
  {
    double tolerance = 2; //2 degree tolerance

    if ((current_angle < (theta+tolerance)) || (current_angle > (theta-tolerance)))
    {
      Robot.m_driveTrain.talonL.set(0);
      Robot.m_driveTrain.talonR.set(0);
      return;
    }
    else if (theta < 0)
    {
      if (current_angle > theta)
      {
        Robot.m_driveTrain.talonR.set(0.5);
        Robot.m_driveTrain.talonL.set(-0.5);
      }
    }
    else if(theta > 0)
    {
      if (current_angle < theta)
      {
        Robot.m_driveTrain.talonR.set(-0.5);
        Robot.m_driveTrain.talonL.set(0.5);
      }
    }
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
