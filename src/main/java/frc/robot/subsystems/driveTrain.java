package frc.robot.subsystems;

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
	public AHRS gyro;
	
	private Solenoid shifter;
	//private DoubleSolenoid shifter;
	Compressor comp;
    
	public static final double MAX_SPEED = 10000;
	public static final int TICKS_PER_METER = 55100;

	private boolean reversed;
	private int gear;
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

		shifter = new Solenoid(RobotMap.shifterCB);
		//shifter = new DoubleSolenoid(RobotMap.shifterPB[0], RobotMap.shifterPB[1]);
		comp = new Compressor(RobotMap.compressor);
		
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
		if (!reversed) {
			diffDrive.arcadeDrive(-speed, -rot);
		}
		else {
			diffDrive.arcadeDrive(speed, -rot);
		}
	}

	public void pathDrive(double leftSpeed, double rightSpeed) {
		if (!reversed) {
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
		this.reversed = reversed;
		if (this.reversed) {
			leftDrive = talonR;
			rightDrive = talonL;
		}
		else {
			leftDrive = talonL;
			rightDrive = talonR;
		}
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
	
    public void gear2() {
		if (!shifter.get()) {
			shifter.set(true);
		}	
		//if (shifter.get() != Value.kReverse) {
			//shifter.set(Value.kReverse);
		//}
	}
	
    public void gear1() {
		if (shifter.get()) {
			shifter.set(false);
		}
		/*
		if (shifter.get() != Value.kForward) {
			shifter.set(Value.kForward);
		}
		*/
	}
	public void toggleGear() {
		/*
		if (shifter.get() == Value.kForward) {
			gear2();
		}
		*/
		if (!shifter.get()) {
			gear2();
		}
		else {
			gear1();
		}
		
	}

	

	public void setLocation(int x, int y) {
		xPos = x;
		yPos = y;
	}
}

	

