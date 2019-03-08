package frc.robot.commands;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class drive extends Command {
    private double driveY;
    private double driveX;
	//private double speedControl;
    public drive() {
    	super("Drive");
        requires(Robot.m_driveTrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        driveY = Robot.m_oi.getDriveY();
        driveX = Robot.m_oi.getDriveX();
        //speedControl = (-Robot.m_oi.getSlider() + 3)/4;
        Robot.m_driveTrain.drive(driveY, driveX);
        Robot.m_habLift.runHabLift(driveY);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    protected void end() { 
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        Robot.m_driveTrain.drive(0, 0);
    }
}
