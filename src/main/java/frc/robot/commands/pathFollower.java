package frc.robot.commands;

import frc.robot.Path;
import frc.robot.Robot;
import frc.robot.subsystems.driveTrain;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class PathFollower extends Command {
	private driveTrain driveT = Robot.m_driveTrain;

	private Notifier follower = new Notifier(this::followPath);
	private Path myPath;
	/*
		+/- : involves left(-) or right(+) loading station (not part of the file name)
		cs/rkt : involves cargo ship(cs) or rocket ship(rkt)
		C/F : deposit area is on close(C) or far(F) side of the field
		number : specific section of the deposit location
		+/- : to(+) or from(-) the loading station

		ex: "+csF2-"
	*/
    public PathFollower(String pathName) {
		super("pathFollower");
		//requires(Robot.m_driveTrain);
		myPath = new Path(pathName);
		//SmartDashboard.putString("Last Path", myPath.getName());
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
		driveT.pathDrive(0, 0);
		driveT.auto = false;
	}

	private void followPath() {
		double[] speeds = myPath.calculateSpeeds(driveT.getEncPosL(), driveT.getEncPosR(), driveT.getAngle());
		driveT.pathDrive(speeds[0], speeds[1]);
	}
}

