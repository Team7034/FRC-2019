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
import frc.robot.subsystems.driveTrain;

public class autoShifter extends Command {
  private driveTrain dt = Robot.m_driveTrain;
  private double lastV = 0;
  public autoShifter() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //UP
    //v > 4m/s, throttle > .8, a > ?
    //DOWN
    //v < 2m/s, throttle < .5
    //v < 8m/s, throttle > .8, large -a
    //v < 2, throttle > .8, high current
    double v = dt.getVelocity();
    double a = (v - lastV)*50;
    lastV = v;
    double t = dt.getThrottle();
    double c = dt.getCurrentDraw();

    SmartDashboard.putNumber("Drive Velocity", v);
    SmartDashboard.putNumber("Drive Acceleration", a);
    SmartDashboard.putNumber("Drive Throttle", t);
    SmartDashboard.putNumber("Drive Current", c);
    /*
    if(v > 1 && t > .8 && a > 2) {
      dt.gear1();
    }
    else if (v < 2 && t < .5 || v < 8 && t > .8 && a < -5 || v < 2 && t > .8 && c > 10) {
      dt.gear2();
    }
    */
    if (Math.abs(v) < .5 && t > .5) {
      dt.setGear(driveTrain.LOW_GEAR);
    }
    /*
    if (Math.abs(t) < .01) {
      dt.setGear(driveTrain.LOW_GEAR);
    }
    */
    if (Math.abs(v) > 1.5 && Math.abs(t) > .1) {
      dt.setGear(driveTrain.HIGH_GEAR);
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
