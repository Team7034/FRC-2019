/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.ColorSensor;
import edu.wpi.first.wpilibj.I2C;
import frc.robot.Robot;

public class seek extends Command {

  public double distance;
  ColorSensor sensor;
  Robot robot;
  String direction;
  boolean line_follow = false;

  private boolean done = false;

  public seek(int d, Robot r, String direc) {

    distance = d;
    direction = direc;
    robot =r;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    sensor = new ColorSensor(I2C.Port.kMXP);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    System.out.println(sensor.grayscale);
    //robot.arcadeDrive(-0.4, 0.0);
    if (sensor.grayscale < 4)
    {
      robot.left_motors.set(0.5);
      robot.right_motors.set(0.5);
    }
    else if (sensor.grayscale > 3)
    {
      robot.left_motors.set(0);
      robot.right_motors.set(0);
      line_follow = true;

    }

    if (line_follow)
    {
      if (distance <= 10)
      {
        robot.left_motors.set(0);
        robot.right_motors.set(0);
        done = true;
      }
      
      else
      {
        if (direction.equalsIgnoreCase("negative")) //change this line to 'positive' if this breaks
        {
        sensor.read();
   // SmartDashboard.putNumber("Value", sensor.grayscale);
    robot.left_motors.set(-.5);
    if (sensor.grayscale > 10)
    {
      robot.right_motors.set(-0.6);
      robot.left_motors.set(-0.55);
    }
    if(sensor.grayscale > 4){
      sensor.read();
      robot.right_motors.set(((sensor.grayscale-4) / 18.25) + .22);
    }else{
      sensor.read();
      robot.right_motors.set(-.3);
    }
    sensor.read();
  }
    if (direction.equalsIgnoreCase("positive"))
{
  sensor.read();
   // SmartDashboard.putNumber("Value", sensor.grayscale);
    robot.right_motors.set(-.5);
    if (sensor.grayscale > 10)
    {
      robot.left_motors.set(-0.6);
      robot.right_motors.set(-0.55);
    }
    if(sensor.grayscale > 4){
      sensor.read();
      robot.left_motors.set(((sensor.grayscale-4) / 18.25) + .22);
    }else{
      sensor.read();
      robot.left_motors.set(-.3);
    }
    sensor.read();
}
      }
    }
  }
  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return done;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    robot.left_motors.set(0);
    robot.right_motors.set(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
