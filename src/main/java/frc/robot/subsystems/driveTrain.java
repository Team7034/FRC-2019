package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 *
 */
public class driveTrain extends Subsystem {

    private DifferentialDrive robot;
    public WPI_TalonSRX talonL;
	public WPI_TalonSRX talonL2;
	public WPI_TalonSRX talonL3;
	public WPI_TalonSRX talonR;
	public WPI_TalonSRX talonR2;
	public WPI_TalonSRX talonR3;
    private Timer timer;
    private AHRS gyro;
    
    public static final double TICKS_PER_IN = 170.0;
	public static final double TICKS_PER_REV = 3600.0;
	public static final double MAX_SPEED = 7000;

    public void initDefaultCommand() {
    	//Drive TalonSRX's
    	talonL = new WPI_TalonSRX(RobotMap.driveL1); //main left motor
		talonL2 = new WPI_TalonSRX(RobotMap.driveL2);
		talonL3 = new WPI_TalonSRX(RobotMap.driveL3);
    	talonR = new WPI_TalonSRX(RobotMap.driveR1); //main right motor
		talonR2 = new WPI_TalonSRX(RobotMap.driveR2);
		talonR3 = new WPI_TalonSRX(RobotMap.driveR3);
		talonL2.follow(talonL);
		talonL3.follow(talonL);
		talonR2.follow(talonR);
		talonR3.follow(talonR);

		//talonR.setInverted(InvertType.InvertMotorOutput);
		
    	talonL.setSelectedSensorPosition(0);
    	talonR.setSelectedSensorPosition(0);
    	
    	robot = new DifferentialDrive(talonL, talonR);
        
        try {
			gyro = new AHRS(SPI.Port.kMXP);
		} catch(RuntimeException e) {
			DriverStation.reportError("Error Instantiating the NavX Micro: " + e.getMessage(), true);
		}
    
        //leftPID = new PIDController(.039,1E-8,0.095,0, gyro, talonL);
        //rightPID = new PIDController(.039,1E-8,0.095,0, gyro, talonR);
        
      	timer = new Timer();
		timer.start();
    }
    
    public void drive(double speed, double rot) {
    	robot.arcadeDrive(-speed, -rot);
    }
    
    public void setTarget(int targetL, int targetR) {
		talonL.set(ControlMode.PercentOutput, 0);
    	talonR.set(ControlMode.PercentOutput, 0);
    	talonL.set(ControlMode.Position, targetL);
    	talonR.set(ControlMode.Position, -targetR);
	}
	
	public double getAngle() {
		return gyro.getAngle();
	}
}

	

