package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.driveTrain;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class pathFollower extends Command {
	private driveTrain dt = Robot.m_driveTrain;

	EncoderFollower left;
	EncoderFollower right;
	
	//Wheel base width = 0.646
	private final double wheel_diameter = 0.104;
	private final double max_velocity = 2.5;
	private final int TICKS_PER_REV = 18000;
	
	public static double kP = 5;
	public static double kI = 0;
	public static double kD = 0.1;
	public static double kA = 0;
	public static double kG = 0.8 * (-1.0/80.0);

	private double offset;

	public String pathName = "simple";

	private Notifier follower = new Notifier(this::followPath);

	//flip is for reusing paths for the rightside loading station (not reverse)
	public boolean flip;
	
	/*
		+/- : involves left(-) or right(+) loading station (not part of the file name)
		cs/rkt : involves cargo ship(cs) or rocket ship(rkt)
		L/R : deposit area is on left(L) or right(R) side of the field
		number : specific section of the deposit location
		+/- : to(+) or from(-) the loading station
	*/
    public pathFollower(String path) {
		super("pathFollower");
		//requires(Robot.m_driveTrain);
		if (path.substring(0,1) == "+") {
			flip = true;
		}
		else {
			flip = false;
		}
		pathName = path.substring(1);
		try {
			offset = dt.getAngle();
		}
		catch(NullPointerException e) {
			offset = 0;
		}
		SmartDashboard.putString("Last Path", path);
	}

    // Called just before this Command runs the first time
    protected void initialize() {
		dt.auto = true;
		//pathName = SmartDashboard.getString("Testing Path", "simple");
		//flip = SmartDashboard.getBoolean("Flip", false);
		kP = SmartDashboard.getNumber("Path P", kP);
		kI = SmartDashboard.getNumber("Path I", kI);
		kD = SmartDashboard.getNumber("Path D", kD);
		kA = SmartDashboard.getNumber("Path A", kA);
		//Generates left and right trajectories from csv files on the roboRIO 
		//DOES NOT WORK WITH LATEST VERSION OF PATHFINDER, USE 2019.1.12

		Trajectory lTrajectory;
		Trajectory rTrajectory;
		lTrajectory = PathfinderFRC.getTrajectory("output/" + pathName + ".left");
		rTrajectory = PathfinderFRC.getTrajectory("output/" + pathName + ".right");
		

		//Creates objects to follow the trajectories
		if (flip) {
			left = new EncoderFollower(rTrajectory);
			right = new EncoderFollower(lTrajectory);
		}
		else {
    		left = new EncoderFollower(lTrajectory);
			right = new EncoderFollower(rTrajectory);
		}
		
		//Configures EncoderFollowers
		left.configureEncoder(dt.getEncPosL(), TICKS_PER_REV, wheel_diameter);
		right.configureEncoder(dt.getEncPosR(), TICKS_PER_REV, wheel_diameter);
		left.configurePIDVA(kP, kI, kD, 1 / max_velocity, kA);
		right.configurePIDVA(kP, kI, kD, 1 / max_velocity, kA);

		//Starts the notifier
		follower.startPeriodic(lTrajectory.get(0).dt);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return left.isFinished() || right.isFinished();
    }

    // Called once after isFinished returns true
    protected void end() {
		follower.stop();
		dt.auto = false;
		dt.setLocation((int) SmartDashboard.getNumber("xPos", 0), (int) SmartDashboard.getNumber("yPos", 0));
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
		follower.stop();
		dt.pathDrive(0, 0);
		dt.auto = false;
	}

	private void followPath() {
		System.out.println(pathName);
		double left_speed = left.calculate(dt.getEncPosL());
		double right_speed = right.calculate(dt.getEncPosR());
		double desired_heading = Pathfinder.r2d(left.getHeading());
		double heading;
		try {
			heading = dt.getAngle() - offset;
		}
		catch(NullPointerException e) {
			heading = desired_heading;
		}
		//heading = desired_heading;
		double heading_difference = Pathfinder.boundHalfDegrees(desired_heading - heading);
		double turn;
		if (flip) {
			turn = -kG * heading_difference;
		}
		else {
			turn = kG * heading_difference;
		}
		dt.pathDrive(left_speed + turn, -right_speed + turn);
	}

	
}

