/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class RocketFinder extends Command {

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


  public RocketFinder() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);

    requires(Robot.m_driveTrain);

    //theta_bryce = bryce_theta;
    //distanceFromRocket = distance;
  }

  turnToAngle test;

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    target = Robot.m_driveTrain.gyro.getYaw() - SmartDashboard.getNumber("BRYCE_YAW", 999);
    SmartDashboard.putNumber("target", target);

    /*current_angle = Robot.m_driveTrain.gyro.getYaw();
    
    test = new turnToAngle(current_angle+90);
    */
    theta_ours = 90 - current_angle + SmartDashboard.getNumber("BRYCE_YAW", 999);
    /*
    if (theta_bryce != 0 && distanceFromRocket != 0)
    {
    theta_ours = 90 + theta_bryce - current_angle;
    SmartDashboard.putNumber("theta_ours", theta_ours);
    
    }
    else if (theta_bryce == 1234560)
    {
      //isFinished();
    }
    else 
      {theta_ours = 9999;
        SmartDashboard.putNumber("theta_ours", theta_ours);

      }
      */
    current_angle = Robot.m_driveTrain.gyro.getYaw();
    height = distanceFromRocket*Math.sin(theta_ours);
    length = distanceFromRocket*Math.cos(theta_ours);
    phi = Math.atan((height-epsilon)/length) * 3.1415/180; //tan inverse = atan = arctan
    rho = 18;

    theta_bryce = SmartDashboard.getNumber("BRYCE_YAW", 0);
    c = Math.sqrt(((length*length)+((height-epsilon))*((height-epsilon))));

      turningAngle = current_angle - theta_bryce;
      SmartDashboard.putNumber("turning angle", turningAngle);

    System.out.println("RocketFinder was initialized");

    SmartDashboard.putNumber("bryce theta", theta_bryce);
    SmartDashboard.putNumber("nathan distance", distanceFromRocket);
    SmartDashboard.putNumber("height from rocket", height);
    SmartDashboard.putNumber("length from rocket", length);
    SmartDashboard.putNumber("nathan phi", phi);
    System.out.println(theta_ours +  "is the angle we are going to turn");
    SmartDashboard.putNumber("theta_ours", theta_ours);
      

    encoderPositionAtStart = (Robot.m_driveTrain.getEncPosR() / 1399.539244); //in inches now
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    System.out.println("we are executing now");
    SmartDashboard.putBoolean("has turned?", hasTurned);

    double error = current_angle - turningAngle;
    if (error >= -1.5 || error <= 1.5)
      {
        hasTurned = true;
      }

      double currentEncoderPosition = (Robot.m_driveTrain.getEncPosR() / 1399.539244); //in inches
      if (hasTurned)
      {
        //if (currentEncoderPosition < (c + rho - currentEncoderPosition))
        if (currentEncoderPosition < (encoderPositionAtStart + (SmartDashboard.getNumber("BRYCE_DISTANCE", 0)*1400) + 1400*30))
        {
          Robot.m_driveTrain.drive(0.5,0);
        }
        else  
          Robot.m_driveTrain.drive(0,0);
        /*
        else if (currentEncoderPosition > (c+rho - currentEncoderPosition))
        {
          //Robot.m_driveTrain.drive((currentEncoderPosition-(c+rho- currentEncoderPosition)/50), 0);
        }
        */
      }


     //Robot.m_driveTrain.talonL.set(0.5);
     // turnToAngle(theta_ours-phi);
     SmartDashboard.putNumber("nathan encoder start", encoderPositionAtStart);
    
     // double target = theta_ours - phi;
       double tolerance = 0; //tolerance degrees
    System.out.println("before if statemetns");
    current_angle = Robot.m_driveTrain.gyro.getYaw();
    
    //Scheduler.getInstance().run();
    //if(!test.isRunning()){
     // Scheduler.getInstance().add(test);
    //}
    SmartDashboard.putNumber("current angle", Robot.m_driveTrain.gyro.getYaw());
    
      //double turn = ((current_angle - target)/110);
      double turn = ((current_angle - turningAngle) * 0.025) + 0.05;
      if (turn > 0.5)
        turn = 0.5;
      else if (turn < -0.5)
        turn = -0.5;
      SmartDashboard.putNumber("turn Srength", turn);
      double turnStrength = turn;
      if (hasTurned == false)
      {
      if (current_angle == 180)
      {
        Robot.m_driveTrain.drive(0.5, 0.5);
      }
      Robot.m_driveTrain.autoDrive(-turnStrength, -turnStrength);
    }
    /*
    if (target > 0)
    {
      current_angle = Robot.m_driveTrain.gyro.getYaw();
      System.out.println("TURNING 1");
      
       if (current_angle < target)
       { 
         System.out.println("TURNING 2"); 
         if (current_angle < (target - tolerance))
         {
         System.out.println("TURNING 3");
          current_angle = Robot.m_driveTrain.gyro.getYaw();
          SmartDashboard.putNumber("current angle", current_angle);
         // Robot.m_driveTrain.drive(0,turnStrength);
         Robot.m_driveTrain.autoDrive(turnStrength, turnStrength);
}
       }
       else if (current_angle > target)
       { 
         if (current_angle > (target + tolerance))
         { System.out.println("TURNING 3");
           current_angle = Robot.m_driveTrain.gyro.getYaw();
           SmartDashboard.putNumber("current angle", current_angle);
           //Robot.m_driveTrain.drive(0,-turnStrength);
           Robot.m_driveTrain.autoDrive(turnStrength, turnStrength);
         }
       }
     }
     if (target < 0)
     {
       if (current_angle < target)
       {  
         if (current_angle < (target - tolerance))
          current_angle = Robot.m_driveTrain.gyro.getYaw();
          SmartDashboard.putNumber("current angle", current_angle);
         //Robot.m_driveTrain.drive(0,turnStrength);
         Robot.m_driveTrain.autoDrive(turnStrength, turnStrength);

         System.out.println("TURNING 3");
       }
       else if (current_angle > target)
       {
         if (current_angle > (target + tolerance))
         {
           current_angle = Robot.m_driveTrain.gyro.getYaw();
           SmartDashboard.putNumber("current angle", current_angle);
           //Robot.m_driveTrain.drive(0,-turnStrength);
           Robot.m_driveTrain.autoDrive(turnStrength, turnStrength);

           System.out.println("TURNING 3");
         }
        }
        
      } 
      */
  }
/*
  protected void turnToAngle(double target)
  {
    double tolerance = 5; //degrees of tolerance
    
    System.out.println("we are running turn to Angle");

    if (target > 0)
    {
      if (current_angle < (target))
      {  
        while (current_angle < (target - tolerance))
         current_angle = Robot.m_driveTrain.gyro.getYaw();
         SmartDashboard.putNumber("current angle", current_angle);
         Robot.m_driveTrain.drive(0,0.5);
      }
      else if (current_angle > target)
      {
        while (current_angle > (target + tolerance))
        {
          current_angle = Robot.m_driveTrain.gyro.getYaw();
          SmartDashboard.putNumber("current angle", current_angle);
          Robot.m_driveTrain.drive(0,-0.5);
        }
      }
    }
    if (target < 0)
    {
      if (current_angle < (target))
      {  
        while (current_angle < (target - tolerance))
         current_angle = Robot.m_driveTrain.gyro.getYaw();
         SmartDashboard.putNumber("current angle", current_angle);
        Robot.m_driveTrain.drive(0,-0.5);
      }
      else if (current_angle > target)
      {
        while (current_angle > (target + tolerance))
        {
          current_angle = Robot.m_driveTrain.gyro.getYaw();
          SmartDashboard.putNumber("current angle", current_angle);
          Robot.m_driveTrain.drive(0,0.5);
        }
      }
    }
  
  //Robot.m_driveTrain.talonL.set(0);
      //Robot.m_driveTrain.talonR.set(0);
      Robot.m_driveTrain.drive(0,0);
      //isFinished();
      return; 
} */

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