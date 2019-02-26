package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 *
 */
public class DriveTrain extends Subsystem {

	private DifferentialDrive robot;
  public WPI_TalonSRX talonL;
	public WPI_TalonSRX talonL2;
	public WPI_TalonSRX talonL3;
	public WPI_TalonSRX talonR;
	public WPI_TalonSRX talonR2;
	public WPI_TalonSRX talonR3;
  private Timer timer;
	private AHRS gyro;
	
	private Solenoid shifter;
    
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

		//brake
		talonL.setNeutralMode(NeutralMode.Coast);
		talonL2.setNeutralMode(NeutralMode.Coast);
		talonL3.setNeutralMode(NeutralMode.Coast);
		talonR.setNeutralMode(NeutralMode.Coast);
		talonR2.setNeutralMode(NeutralMode.Coast);
		talonR3.setNeutralMode(NeutralMode.Coast);

		//safety
		talonL.setSafetyEnabled(false);
		talonL2.setSafetyEnabled(false);
		talonL3.setSafetyEnabled(false);
		talonR.setSafetyEnabled(false);
		talonR2.setSafetyEnabled(false);
		talonR3.setSafetyEnabled(false);

		//shifting
		shifter = new Solenoid(RobotMap.shifter);
		
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
  }
    
  public void drive(double speed, double rot) {
		if (Robot.forward) {
			robot.arcadeDrive(speed, -rot);
		}
		else {
			robot.arcadeDrive(-speed, -rot);
		}
	}
    
  /*public void setTarget(int targetL, int targetR) {
		talonL.set(ControlMode.PercentOutput, 0);
    talonR.set(ControlMode.PercentOutput, 0);
    talonL.set(ControlMode.Position, targetL);
    talonR.set(ControlMode.Position, -targetR);
	}
	
	public double getAngle() {
		return gyro.getAngle();
	}

	public void zero() {
		talonL.setSelectedSensorPosition(0);
		talonR.setSelectedSensorPosition(0);
		gyro.zeroYaw();
	}

	public double getResistance() {
		double resistanceL = Math.abs(MAX_SPEED*talonL.getMotorOutputPercent()/talonL.getSelectedSensorVelocity());
		double resistanceR = Math.abs(MAX_SPEED*talonR.getMotorOutputPercent()/talonR.getSelectedSensorVelocity());
		return (resistanceL + resistanceR)/2;
	}*/

	public void highGear(boolean gear){
		//sets gearbox to high gear or low gear
		shifter.set(gear);
	}

	private void autoShift(){
		//shift UP condition
		//velocity > set param (4ms), throttle > set param (80%), accel > set param (?mss)

		//shift DOWN condition
		//velocity < set param (2ms), throttle < set param (50%)

		//shift DOWN push condition
		//velocity < set param (8ms), throttle > set param (80%), large negative accel

		//shift DOWN push condition
		//velocity < set param (2ms), throttle > set param (80%), high current draw
	}
}