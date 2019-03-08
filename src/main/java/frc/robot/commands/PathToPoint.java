/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Path;
import frc.robot.Robot;
import frc.robot.subsystems.driveTrain;

public class PathToPoint extends Command {
  private driveTrain driveT = Robot.m_driveTrain;

	public String pathName = "simple";

	private Notifier follower = new Notifier(this::followPath);
	private Path myPath;

	//flip is for reusing paths for the rightside loading station (not reverse)
	public boolean flip;
	
	/*
		+/- : involves left(-) or right(+) loading station (not part of the file name)
		cs/rkt : involves cargo ship(cs) or rocket ship(rkt)
		L/R : deposit area is on left(L) or right(R) side of the field
		number : specific section of the deposit location
		+/- : to(+) or from(-) the loading station
	*/
  public PathToPoint(double x, double y, double heading) {
		super("pathFollower");
		//requires(Robot.m_driveTrain);
		myPath = new Path(x, y, heading);
	}

      // Called just before this Command runs the first time
  protected void initialize() {
		driveT.auto = true;

		//Configures EncoderFollowers
		myPath.configSensors(driveT.getEncPosL(), driveT.getEncPosR(), driveT.getAngle());

		//Starts the notifier
    follower.startPeriodic(myPath.getDT());
  }

  // Called repeatedly when this Command is scheduled to run
  protected void execute() {
    	
  }

  // Make this return true when this Command no longer needs to run execute()
  protected boolean isFinished() {
    return myPath.getLeftEncFollower().isFinished() || myPath.getRightEncFollower().isFinished();
  }

    // Called once after isFinished returns true
  protected void end() {
		follower.stop();
		driveT.auto = false;
    driveT.setLocation((int) SmartDashboard.getNumber("xPos", 0), (int) SmartDashboard.getNumber("yPos", 0));
    myPath.reset();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  protected void interrupted() {
		follower.stop();
		driveT.autoDrive(0, 0);
		driveT.auto = false;
	}

	private void followPath() {
		double[] speeds = myPath.calculateSpeeds(driveT.getEncPosL(), driveT.getEncPosR(), driveT.getAngle());
		driveT.autoDrive(speeds[0], speeds[1]);
	}
}
