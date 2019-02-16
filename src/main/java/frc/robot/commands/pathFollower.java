package frc.robot.commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class pathFollower extends Command {
	EncoderFollower left;
	EncoderFollower right;
	
	//Wheel base width = 0.646
	private final double wheel_diameter = 0.1;
	private final double max_velocity = 1;
	private final int TICKS_PER_REV = 18000;
	
	private final double P = 1;
	private final double I = 0;
	private final double D = 0.1;
	private final double A = 0;

	private String pathName = "";

	private Notifier follower;

	public static int x;
	public static int y;
	
    public pathFollower(int newX, int newY) {
    	super("pathFollower");
		requires(Robot.m_driveTrain);
	}
	public pathFollower(String pathName) {
    	super("pathFollower");
		requires(Robot.m_driveTrain);
		this.pathName = pathName;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
		pathName = SmartDashboard.getString("Path Name", "simple");

		//Generates left and right trajectories from csv files on the roboRIO
		Trajectory lTrajectory = PathfinderFRC.getTrajectory(pathName + ".left");
		Trajectory rTrajectory = PathfinderFRC.getTrajectory(pathName + ".right");
		
		//Creates objects to follow the trajectories
    	left = new EncoderFollower(lTrajectory);
		right = new EncoderFollower(rTrajectory);
		
		

		//Configures EncoderFollowers
		left.configureEncoder(Robot.m_driveTrain.talonL.getSelectedSensorPosition(), TICKS_PER_REV, wheel_diameter);
		right.configureEncoder(-Robot.m_driveTrain.talonR.getSelectedSensorPosition(), TICKS_PER_REV, wheel_diameter);
		left.configurePIDVA(P, I, D, 1 / max_velocity, A);
		right.configurePIDVA(P, I, D, 1 / max_velocity, A);

		//Creates and starts the notifier
		follower = new Notifier(this::followPath);
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
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
		follower.stop();
		Robot.m_driveTrain.talonL.set(0);
		Robot.m_driveTrain.talonR.set(0);
	}

	private void followPath() {
		double left_speed = left.calculate(Robot.m_driveTrain.talonL.getSelectedSensorPosition());
		double right_speed = right.calculate(-Robot.m_driveTrain.talonR.getSelectedSensorPosition());
		double desired_heading = Pathfinder.r2d(left.getHeading());
		double heading;
		try {
			heading = Robot.m_driveTrain.getAngle();
		} 
		catch(NullPointerException e) {
			heading = desired_heading;
		}
		double heading_difference = Pathfinder.boundHalfDegrees(desired_heading - heading);
		double turn =  -0.01 * heading_difference;
		Robot.m_driveTrain.talonL.set(left_speed + turn);
		Robot.m_driveTrain.talonR.set(-right_speed + turn);
			
    	//SmartDashboard.putNumber("VelocityR", Robot.m_driveTrain.talonR.getSelectedSensorVelocity());
	}
}

