package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.driveTrain;
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
	private final double wheel_diameter = 0.1;
	private final double max_velocity = 1;
	private final int TICKS_PER_REV = 18000;
	
	public static double P = 1;
	public static double I = 0;
	public static double D = 0;
	public static double A = 0;

	private String pathName = "";

	private Notifier follower = new Notifier(this::followPath);

	public static int[] location = new int[]{0, 0};
	private int[] destination;
	//flip is for reusing paths for the rightside loading station (not reverse)
	private boolean flip = false;
	
    public pathFollower(int newX, int newY) {
    	super("pathFollower");
		requires(Robot.m_driveTrain);
		flip = false;
		String loc = getPosName(location[0], location[1]);
		pathName = loc + getPosName(newX, newY);
		if (loc == "") {
			pathName += "-";
		}
		destination = new int[]{newX, newY};
	}
	public pathFollower(String pathName) {
    	super("pathFollower");
		requires(Robot.m_driveTrain);
		Robot.auto = true;
		flip = SmartDashboard.getBoolean("Flip", false);
		this.pathName = pathName;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
		pathName = SmartDashboard.getString("Path Name", "simple");

		//Generates left and right trajectories from csv files on the roboRIO
		Trajectory lTrajectory = PathfinderFRC.getTrajectory(pathName + ".left");
		Trajectory rTrajectory = PathfinderFRC.getTrajectory(pathName + ".right");
		
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
		left.configurePIDVA(P, I, D, 1 / max_velocity, A);
		right.configurePIDVA(P, I, D, 1 / max_velocity, A);

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
		Robot.auto = false;
		location = destination;
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
		follower.stop();
		dt.pathDrive(0, 0);
		Robot.auto = false;
	}

	private void followPath() {
		double left_speed = left.calculate(dt.getEncPosL());
		double right_speed = right.calculate(dt.getEncPosR());
		double desired_heading = Pathfinder.r2d(left.getHeading());
		double heading;
		try {
			heading = Robot.m_driveTrain.getAngle();
		}
		catch(NullPointerException e) {
			heading = desired_heading;
		}
		double heading_difference = Pathfinder.boundHalfDegrees(desired_heading - heading);
		double turn;
		if (flip) {
			turn = 0.01 * heading_difference;
		}
		else {
			turn = -0.01 * heading_difference;
		}
		Robot.m_driveTrain.pathDrive(left_speed + turn, -right_speed + turn);
	}

	private String getPosName(int xPos, int yPos) {
		switch(xPos) {
			case 0: 
				if(yPos >= 1) {
					return "rktL" + yPos;
				}
				return "";
			case 1:
				return "csL" + (yPos+1);
			case 2:
				return "csR" + (yPos+1);
			case 3:
				if(yPos >= 1) {
					return "rktR" + yPos;
				}
				else {
					flip = true;
					return "";
				}
			default: 
				return "";
		}
	}
}

