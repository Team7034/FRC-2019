/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.driveTrain;
import frc.robot.subsystems.pneumatics;
import frc.robot.subsystems.arm;
import frc.robot.commands.drive;
import frc.robot.commands.mainAuto;
import frc.robot.commands.moveArm;
//import frc.robot.commands.seek;
//import frc.robot.commands.turnToAngle;
import frc.robot.commands.rocketFinder;

//import edu.wpi.first.wpilibj.PIDController;
//import edu.wpi.first.wpilibj.PIDOutput;
//import com.kauailabs.navx.frc.AHRS;

//import edu.wpi.first.wpilibj.TimedRobot;
//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import javax.lang.model.util.ElementScanner6;

//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
//import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import java.util.*;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static OI m_oi;
  public static driveTrain m_driveTrain = new driveTrain();
  public static pneumatics m_pneumatics = new pneumatics();
  public static arm m_arm = new arm();

  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  public SpeedControllerGroup left_motors;
  public SpeedControllerGroup right_motors;
  
  ColorSensor sensor;
  
  DifferentialDrive robot;
  Joystick stick;

  Spark front_left;
	Spark back_left;	
	Spark front_right;
  Spark back_right;

  Deque<String> deque;

  

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_oi = new OI();
    m_chooser.setDefaultOption("Default Auto", new mainAuto());
    //chooser.addOption("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", m_chooser);
    Scheduler.getInstance().add(new rocketFinder(0, 0));

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
    //System.out.println(sensor.grayscale);
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    //Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_chooser.getSelected();


    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    Scheduler.getInstance().add(new drive());
    Scheduler.getInstance().add(new moveArm());
    
    front_left = new Spark(0);
		back_left = new Spark(1);
		left_motors = new SpeedControllerGroup(front_left, back_left);
		
		front_right = new Spark(2);
		back_right = new Spark(3);
		right_motors = new SpeedControllerGroup(front_right, back_right);
		
		robot = new DifferentialDrive(left_motors, right_motors);
		stick = new Joystick(0);
    stick.setThrottleChannel(3);

    sensor = new ColorSensor(I2C.Port.kOnboard);
    deque = new LinkedList<String>();

  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
    SmartDashboard.putNumber("PositionL", m_driveTrain.talonL.getSelectedSensorPosition());
    SmartDashboard.putNumber("PositionR", m_driveTrain.talonR.getSelectedSensorPosition());
    SmartDashboard.putNumber("VelocityL", m_driveTrain.talonL.getSelectedSensorVelocity());
    SmartDashboard.putNumber("VelocityR", m_driveTrain.talonR.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Gyro", m_driveTrain.getAngle());

    double speed = -(stick.getThrottle()+1)/2;
    robot.arcadeDrive(-stick.getY()*speed, -stick.getX()*speed);
  }


  @Override
  public void testInit(){
    //Scheduler.getInstance().add(new seek(100, this, "negative"));

    //double yaw = SmartDashboard.getNumber("tapeYaw", 0);
    //Scheduler.getInstance().add(new turnToAngle(30, this));
    //Scheduler.getInstance().add(new turnToAngle(yaw, this));
  }
  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    Scheduler.getInstance().run();
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

    
    /*
    sensor.read();
    SmartDashboard.putNumber("Value", sensor.grayscale);
    left_motors.set(-.5);
    if(sensor.grayscale > 4){
      right_motors.set(((sensor.grayscale-4) / 16.25) + .22);
    }else{
      right_motors.set(-.3);
      
    }
    */

    
  }
}
