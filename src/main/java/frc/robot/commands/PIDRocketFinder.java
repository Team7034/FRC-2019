/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
//import sun.security.krb5.internal.crypto.crc32;

public class PIDRocketFinder extends Command {

  public double theta_bryce; //degrees and backwards (left = pos, right = neg)
  public double theta_ours = 9999; //degrees (left = neg, right = pos)
  public double current_angle; //degrees
  public double distanceFromRocket; //inches
  public double heightFromRocket; //inches
  public double lengthFromRocket; //inches
  public double epsilon = 18; //inches, variable target distance from rocket
  public double phi; //offset of angle that we want to turn
  public double height; //inches, vertical distance from rocket in xy plane
  public double length; //inches, horizontonal distance from rocket in xy plane
  public double rho; //inches, distance from front of robot to center, 16"
  public double turningAngle; //angle that we want to turn, equal to theta_ours - phi

  private double target; //what we want to turn to;
  public double c; //base diagonal distance for robot to travel along path 

  public boolean hasTurned;
  public double encoderPositionAtStart;
  PIDController leftPID; 

  public PIDRocketFinder() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);

    requires(Robot.m_driveTrain);

    //theta_bryce = bryce_theta;
    //distanceFromRocket = distance;
  }

  
  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    hasTurned = false;

    
    target = Robot.m_driveTrain.gyro.getYaw() - SmartDashboard.getNumber("BRYCE_YAW", 0);

    leftPID = new PIDController(.05,0,0.000,0, Robot.m_driveTrain.gyro, Robot.m_driveTrain.leftDrive);
    leftPID.setAbsoluteTolerance(1);
    leftPID.setSetpoint(target);
    leftPID.enable();
    SmartDashboard.putNumber("PID target", target);
    encoderPositionAtStart = (Robot.m_driveTrain.getEncPosR() / 1399.539244); //in inches now
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    double yaw = SmartDashboard.getNumber("BRYCE_YAW", 0);
    current_angle = Robot.m_driveTrain.gyro.getYaw();

    //leftPID.setSetpoint(current_angle - yaw);
    //System.out.println(current_angle-yaw + " this is the angle target");
    //System.out.println("this is the current angle: " + current_angle);
    //System.out.println("this is the current pid value" + leftPID.get());
    Robot.m_driveTrain.talonR.set(leftPID.get());
    Robot.m_driveTrain.talonR.set(leftPID.get());
    SmartDashboard.putNumber("Pid strength output", leftPID.get());

    if (leftPID.onTarget())
    {
      leftPID.disable();
      hasTurned = true;
    }
     




    SmartDashboard.putBoolean("has turned?", hasTurned);

    double currentEncoderPosition = (Robot.m_driveTrain.getEncPosR() / 1399.539244); //in inches
    if (hasTurned == true)
    {
      //if (currentEncoderPosition < (c + rho - currentEncoderPosition))
      if (currentEncoderPosition < (encoderPositionAtStart + (SmartDashboard.getNumber("BRYCE_DISTANCE", 0)*1400) + 1400*30))
      {
        //Robot.m_driveTrain.drive(0.5,0);
      }
      else  
        Robot.m_driveTrain.drive(0,0);
    }


     SmartDashboard.putNumber("nathan encoder start", encoderPositionAtStart);
    
      current_angle = Robot.m_driveTrain.gyro.getYaw();
    
    
    SmartDashboard.putNumber("current angle", Robot.m_driveTrain.gyro.getYaw());
    
      
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    leftPID.disable();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    leftPID.disable();    
  }
  
}