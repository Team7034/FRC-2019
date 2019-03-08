package frc.robot.commands;

import frc.robot.Path;
import frc.robot.Robot;
import frc.robot.subsystems.driveTrain;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;
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
    public PathFollower(Path path) {
		super("pathFollower");
		//requires(Robot.m_driveTrain);
		this.myPath = path;
	}

    // Called just before this Command runs the first time
    protected void initialize() {
		driveT.auto = true;

		//Configures EncoderFollowers
		myPath.configSensors(driveT.getEncPosL(), driveT.getEncPosR(), driveT.getAngle());
		
		double kP = SmartDashboard.getNumber("Path P", myPath.kP);
		double kI = SmartDashboard.getNumber("Path I", myPath.kI);
		double kD = SmartDashboard.getNumber("Path D", myPath.kD);
        double kA = SmartDashboard.getNumber("Path A", myPath.kA);
        double kG = SmartDashboard.getNumber("Path G", myPath.kG);
		myPath.configPIDAG(kP, kI, kD, kA, kG);
		
		//Starts the notifier
		follower.startPeriodic(myPath.getDT());
		System.out.println("_Starting path");
		//SmartDashboard.putString("Last Path", myPath.getName());
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return myPath.getLeftEncFollower().isFinished() || myPath.getRightEncFollower().isFinished() || !driveT.auto;
    }

    // Called once after isFinished returns true
    protected void end() {
		follower.stop();
		driveT.autoDrive(0, 0);
		driveT.setLocation(myPath.getX(), myPath.getY());
		myPath.reset();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
		follower.stop();
		driveT.autoDrive(0, 0);
	}

	private void followPath() {
		double[] speeds = myPath.calculateSpeeds(driveT.getEncPosL(), driveT.getEncPosR(), driveT.getAngle());
		driveT.autoDrive(speeds[0], speeds[1]);
	}
}

