package frc.robot.commands;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class Drive extends Command {
	//private double speedControl;
  public Drive() {
    //super("drive");
    requires(Robot.m_drive_train);
    //Zeroes drive encoders
    //Robot.m_driveTrain.talonL.getSensorCollection().setQuadraturePosition(0, 30);
    //Robot.m_driveTrain.talonR.getSensorCollection().setQuadraturePosition(0, 30);
  }

  // Called just before this Command runs the first time
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  protected void execute() {
    Robot.m_drive_train.drive(Robot.m_oi.getDriveY(), Robot.m_oi.getDriveX());
  }

  // Make this return true when this Command no longer needs to run execute()
  protected boolean isFinished() {
      return true; //!(Math.abs(driveX) > 0.01 || Math.abs(driveY) > 0.01);
  }

  // Called once after isFinished returns true
  protected void end() { 
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  protected void interrupted() {
    //Robot.m_drive_train.drive(0, 0);
  }
}