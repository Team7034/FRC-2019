package frc.robot.commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class mainAuto extends Command {
	EncoderFollower left;
	EncoderFollower right;
	
	//Wheel base width = 0.646
	private final double wheel_diameter = 0.1;
	private final double max_velocity = 1;
	private final int TICKS_PER_REV = 18000;
	
	private final double P = 1;
	private final double I = 0;
	private final double D = 0;
	private final double A = 0;

	private String pathName = "simple";

	private Notifier follower;
	
    public mainAuto() {
    	super("mainAuto");
		requires(Robot.m_driveTrain);
		requires(Robot.m_pneumatics);
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
		Robot.m_pneumatics.retractShifter();

		//Generates left and right trajectories from csv files on the roboRIO
		Trajectory lTrajectory = PathfinderFRC.getTrajectory(pathName + ".left");
		Trajectory rTrajectory = PathfinderFRC.getTrajectory(pathName + ".right");
		
		//Creates objects to follow the trajectories
    	left = new EncoderFollower(lTrajectory);
		right = new EncoderFollower(rTrajectory);
		
		//Zeroes encoders
		Robot.m_driveTrain.talonL.setSelectedSensorPosition(0);
		Robot.m_driveTrain.talonL.setSelectedSensorPosition(0);

		//Configures EncoderFollowers
		left.configureEncoder(Robot.m_driveTrain.talonL.getSelectedSensorPosition(), TICKS_PER_REV, wheel_diameter);
		right.configureEncoder(Robot.m_driveTrain.talonR.getSelectedSensorPosition(), TICKS_PER_REV, wheel_diameter);
		left.configurePIDVA(P, I, D, 1 / max_velocity, A);
		right.configurePIDVA(P, I, D, 1 / max_velocity, A);

		//Creates and starts the notifier
		follower = new Notifier(this::followPath);
		follower.startPeriodic(lTrajectory.get(0).dt);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	//double l = left.calculate(Robot.m_driveTrain.talonL.getSelectedSensorPosition());
    	//double r = right.calculate(Robot.m_driveTrain.talonR.getSelectedSensorPosition());
    	
    	//double gyro_heading = (gyro);
    	//double desired_heading = Pathfinder.r2d(left.getHeading());
    	
    	//double angleDifference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);
    	//double turn = 0.8 * (-1.0/80.0) * angleDifference;
    	
    	//Robot.m_driveTrain.talonL.set(l);
    	//Robot.m_driveTrain.talonR.set(r);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return left.isFinished() || right.isFinished();
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
	}

	private void followPath() {
		if (left.isFinished() || right.isFinished()) {
		  	follower.stop();
		} 
		else {
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
		  	double turn =  0.8 * (-1.0/80.0) * heading_difference;
		  	Robot.m_driveTrain.talonL.set(left_speed + turn);
			Robot.m_driveTrain.talonR.set(right_speed - turn);
			//SmartDashboard.putNumber("PositionL", Robot.m_driveTrain.talonL.getSelectedSensorPosition());
    		//SmartDashboard.putNumber("PositionR", Robot.m_driveTrain.talonR.getSelectedSensorPosition());
    		//SmartDashboard.putNumber("VelocityL", Robot.m_driveTrain.talonL.getSelectedSensorVelocity());
    		//SmartDashboard.putNumber("VelocityR", Robot.m_driveTrain.talonR.getSelectedSensorVelocity());
		}
	}
}

