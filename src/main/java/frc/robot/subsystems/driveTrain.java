package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class driveTrain extends Subsystem {

    private DifferentialDrive diffDrive;
    public WPI_TalonSRX talonL;
	private WPI_TalonSRX talonL2;
	private WPI_TalonSRX talonL3;
	public WPI_TalonSRX talonR;
	private WPI_TalonSRX talonR2;
	private WPI_TalonSRX talonR3;
	public WPI_TalonSRX leftDrive;
	public WPI_TalonSRX rightDrive;
	public AHRS gyro;
	
	public Solenoid shifter;
	//private DoubleSolenoid shifter;
	public Compressor comp;
    
	public static final double MAX_SPEED = 10000;
	public static final int TICKS_PER_METER = 55100;
	public static final boolean LOW_GEAR = true;
	public static final boolean HIGH_GEAR = false;

	public boolean auto;
	public int xPos = 0;
	public int yPos = 0;

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

		talonL.setNeutralMode(NeutralMode.Brake);
		talonL2.setNeutralMode(NeutralMode.Brake);
		talonL3.setNeutralMode(NeutralMode.Brake);
		talonR.setNeutralMode(NeutralMode.Brake);
		talonR2.setNeutralMode(NeutralMode.Brake);
		talonR3.setNeutralMode(NeutralMode.Brake);

		auto = false;

		//Intitializes reversed, rightDrive, leftDrive, robot
		setReversed(true);
		diffDrive = new DifferentialDrive(talonL, talonR);

		comp = new Compressor(RobotMap.compressor);
		shifter = new Solenoid(RobotMap.shifter);
		//setGear(false); //true = low gear

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
		
		if (!Robot.forward) {
			diffDrive.arcadeDrive(-speed, -rot);
		}
		else {
			diffDrive.arcadeDrive(speed, -rot);
		}
	}

	public void autoDrive(double leftSpeed, double rightSpeed) {
		if (!Robot.forward) {
			diffDrive.tankDrive(leftSpeed, -rightSpeed);
		}
		else {
			diffDrive.tankDrive(rightSpeed, -leftSpeed);
		}
		
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
		Robot.forward = reversed;
		if (Robot.forward) {
			leftDrive = talonR;
			rightDrive = talonL;
		}
		else {
			leftDrive = talonL;
			rightDrive = talonR;
		}
	}

	public boolean getReversed() {
		return Robot.forward;
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

	public double getVelocity() {
		return (rightDrive.getSelectedSensorVelocity()-leftDrive.getSelectedSensorVelocity())/.1/2/TICKS_PER_METER;
	}
	public double getThrottle() {
		return (rightDrive.getMotorOutputPercent()-leftDrive.getMotorOutputPercent())/2;
	}
	public double getCurrentDraw() {
		return (rightDrive.getOutputCurrent()+leftDrive.getOutputCurrent())/2;
	}
	
	public void compressorOn(boolean on) {
    	comp.setClosedLoopControl(on);
	}
	
	//low gear is true
    public void setGear(boolean gear) {
		if (gear == HIGH_GEAR) {
			shifter.set(HIGH_GEAR);
		}
		else {
			shifter.set(LOW_GEAR);
		}
	}

	public void toggleGear() {
		setGear(!shifter.get());
	}

	public boolean inHighGear() {
		return shifter.get() == HIGH_GEAR;
	}

	public void setLocation(int x, int y) {
		xPos = x;
		yPos = y;
	}
}

	

