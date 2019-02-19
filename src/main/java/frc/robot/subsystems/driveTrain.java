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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class driveTrain extends Subsystem {

    private DifferentialDrive diffDrive;
    private WPI_TalonSRX talonL;
	private WPI_TalonSRX talonL2;
	private WPI_TalonSRX talonL3;
	private WPI_TalonSRX talonR;
	private WPI_TalonSRX talonR2;
	private WPI_TalonSRX talonR3;
	public WPI_TalonSRX leftDrive;
	public WPI_TalonSRX rightDrive;
    private AHRS gyro;
    
    public static final double TICKS_PER_IN = 170.0;
	public static final double TICKS_PER_REV = 3600.0;
	public static final double MAX_SPEED = 7000;

	private boolean reversed;

    public void initDefaultCommand() {
    	//Drive TalonSRX's
    	talonL = new WPI_TalonSRX(RobotMap.driveL1); //main left motor
		talonL2 = new WPI_TalonSRX(RobotMap.driveL2);
		talonL3 = new WPI_TalonSRX(RobotMap.driveL3);
		talonL2.follow(talonL);
		talonL3.follow(talonL);
		
		talonR = new WPI_TalonSRX(RobotMap.driveR1); //main right motor
		talonR2 = new WPI_TalonSRX(RobotMap.driveR2);
		talonR3 = new WPI_TalonSRX(RobotMap.driveR3);
		talonR2.follow(talonR);
		talonR3.follow(talonR);
		
		//Intitializes reversed, rightDrive, leftDrive, robot
		setReversed(false);
		
		//Initializes gyro
        try {
			gyro = new AHRS(SPI.Port.kMXP);
		} catch(RuntimeException e) {
			DriverStation.reportError("Error Instantiating the NavX Micro: " + e.getMessage(), true);
		}

    	//Sets encoders and gyro to zero
		zero();

        //leftPID = new PIDController(.039,1E-8,0.095,0, gyro, talonL);
		//rightPID = new PIDController(.039,1E-8,0.095,0, gyro, talonR);
    }
    
    public void drive(double speed, double rot) {
		diffDrive.arcadeDrive(-speed, -rot);
	}

	public void pathDrive(double leftPercentage, double rightPercentage) {
		leftDrive.set(leftPercentage);
		rightDrive.set(rightPercentage);
	}

	public int getEncPosL() {
		return leftDrive.getSelectedSensorPosition();
	}

	public int getEncPosR() {
		return -rightDrive.getSelectedSensorPosition();
	}

	public double getSpeed() {
		return (leftDrive.getSelectedSensorVelocity() - rightDrive.getSelectedSensorVelocity())/2.0;
	}

	public void setReversed(boolean reversed) {
		this.reversed = reversed;
		if (this.reversed) {
			leftDrive = talonR;
			rightDrive = talonL;
		}
		else {
			leftDrive = talonL;
			rightDrive = talonR;
		}
		diffDrive = new DifferentialDrive(leftDrive, rightDrive);
	}

	public boolean getReversed() {
		return reversed;
	}
    
    public void setTarget(int targetL, int targetR) {
		leftDrive.set(ControlMode.PercentOutput, 0);
    	rightDrive.set(ControlMode.PercentOutput, 0);
    	leftDrive.set(ControlMode.Position, targetL);
    	rightDrive.set(ControlMode.Position, -targetR);
	}
	
	public double getAngle() {
		return gyro.getAngle();
	}

	public void zero() {
		leftDrive.setSelectedSensorPosition(0);
		rightDrive.setSelectedSensorPosition(0);
		gyro.zeroYaw();
	}

	public double getResistance() {
		double resistanceL = Math.abs(MAX_SPEED*leftDrive.getMotorOutputPercent()/leftDrive.getSelectedSensorVelocity());
		double resistanceR = Math.abs(MAX_SPEED*rightDrive.getMotorOutputPercent()/rightDrive.getSelectedSensorVelocity());
		return (resistanceL + resistanceR)/2;
	}
}

	

