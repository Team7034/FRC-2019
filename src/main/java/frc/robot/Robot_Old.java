/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import javax.lang.model.util.ElementScanner6;
//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
//import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot_Old extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  Spark front_left;
	Spark back_left;	
	Spark front_right;
  Spark back_right;
  
  //DigitalInput sensor;
  ColorSensor sensor;
  int rightTurnCycles;
  int leftTurnCycles;
  boolean lastLeft;

  int forwardCycles;
	
	SpeedControllerGroup left_motors;
	SpeedControllerGroup right_motors;
  
  DifferentialDrive robot;
  Joystick stick;
  LineFollower lineOutput;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

  forwardCycles = 0;    

   //sensor = new DigitalInput(0);
   sensor = new ColorSensor(I2C.Port.kOnboard);
    //lineOutput = new LineFollower(sensor);

    front_left = new Spark(0);
		back_left = new Spark(1);
		left_motors = new SpeedControllerGroup(front_left, back_left);
		
		front_right = new Spark(2);
		back_right = new Spark(3);
		right_motors = new SpeedControllerGroup(front_right, back_right);
		
		robot = new DifferentialDrive(left_motors, right_motors);
		stick = new Joystick(0);
    stick.setThrottleChannel(3);

    rightTurnCycles = 0;
    leftTurnCycles = 0;
    lastLeft= false; //the line follower starts by going right

    System.out.println("completed intialization! -- Nathan");
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

    if(stick.getTopPressed() == true)
    {
    sensor.read();
    System.out.println("red: " + sensor.red);
    System.out.println("blue: " + sensor.blue);
    System.out.println("green: " + sensor.green);
    System.out.println("grayscale: " + sensor.grayscale);

    }
      }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // autoSelected = SmartDashboard.getString("Auto Selector",
    // defaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
      robot.arcadeDrive(.6, .6);
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        double speed = -(stick.getThrottle()+1)/2;
        robot.arcadeDrive(-stick.getY()*speed, -stick.getX()*speed);
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    double speed = -(stick.getThrottle()+1)/2;
    robot.arcadeDrive(-stick.getY()*speed, -stick.getX()*speed);
    /*
    
    sensor.read();
    SmartDashboard.putNumber("Value", sensor.grayscale);
    left_motors.set(-.5);
    if (sensor.grayscale > 10)
    {
      right_motors.set(-0.6);
      left_motors.set(-0.55);
    }
    if(sensor.grayscale > 4){
      sensor.read();
      right_motors.set(((sensor.grayscale-4) / 18.25) + .22);
    }else{
      sensor.read();
      right_motors.set(-.3);
    }
    sensor.read();
    */
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {

    /**
     * power to overcome mu_s ≈ .22 (2-cim bunnybot gearbox)
     * thus,
     * y = mx + b where x ∈ [-1, 1] & y ∈ [-1, 1]
     * m = 1-mu_s and b = mu_s
     * 
     * To find the proper power, p, based on target power, p_t,
     * given color sensor value v such that v ∈ [0, 10]...
     * p = v / (10 / ((p_t - b) / m)) + b
     * 
     * This equation was derived by...
     * p_t = y
     * solve for x. This is the amount of power administered when the v = 10
     * x = 10 / k
     * solve for k. This is what you must divide v by to find the appropriate power.
     * 
     * Ex. Have the left motor run at 50% power when it is fully over the line. mu_s = .22
     * .5 = .78x + .22
     * x = .3589
     * .3589 = 10 / k
     * k = 27.86
     * left_motors.set((v/k)+.22)
     */
    sensor.read();
    SmartDashboard.putNumber("Value", sensor.grayscale);
    left_motors.set(-.5);
    if(sensor.grayscale > 4){
      right_motors.set(((sensor.grayscale-4) / 20) + .22);
    }else{
      right_motors.set(-.3);
    }
  }
}