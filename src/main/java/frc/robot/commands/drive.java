package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.driveTrain;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class drive extends Command {
	//private double speedControl;
    public drive() {
    	super("drive");
        requires(Robot.m_driveTrain);
        //Zeroes drive encoders
    	//Robot.m_driveTrain.talonL.getSensorCollection().setQuadraturePosition(0, 30);
    	//Robot.m_driveTrain.talonR.getSensorCollection().setQuadraturePosition(0, 30);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.m_pneumatics.extendShifter();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	//speedControl = (-Robot.m_oi.getSlider() + 3)/4;
    	Robot.m_driveTrain.drive(Robot.m_oi.getDriveY(), Robot.m_oi.getDriveX());
    	//Robot.m_driveTrain.vDrive(Robot.m_oi.getDriveY(), Robot.m_oi.getDriveX(), false);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {  	
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
