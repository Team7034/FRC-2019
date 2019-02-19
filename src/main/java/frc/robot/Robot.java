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
import frc.robot.subsystems.*;
import frc.robot.commands.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static OI m_oi;
  //public static arm m_arm = new arm();
  public static driveTrain m_driveTrain = new driveTrain();
  //public static lift m_lift = new lift();
  public static shifter m_shifter = new shifter();
  //public static claw m_claw = new claw();
  public static habLift m_habLift = new habLift();

  public static DashboardListener listener = new DashboardListener();


  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  public static boolean auto;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_oi = new OI();
    m_chooser.setDefaultOption("Default Auto", new pathFollower("simple"));
    //chooser.addOption("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", m_chooser);
    SmartDashboard.putString("Testing Path", "simple");
    //new LaunchPadListener();
    SmartDashboard.putBoolean("automatic", false);
    SmartDashboard.putBoolean("highGear", true);
    SmartDashboard.putBoolean("forward", true);
    SmartDashboard.putNumber("xPos", 0);
    SmartDashboard.putNumber("yPos", 0);
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
    Scheduler.getInstance().run();
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
    //Scheduler.getInstance().add(new moveArm());
    //Scheduler.getInstance().add(new moveLift());
    listener.run();
    auto = false;
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    if(!auto) {
      Scheduler.getInstance().add(new drive());
    }
    Scheduler.getInstance().run();
    SmartDashboard.putNumber("PositionL", m_driveTrain.getEncPosL());
    SmartDashboard.putNumber("PositionR", m_driveTrain.getEncPosL());
    SmartDashboard.putNumber("Speed", m_driveTrain.getSpeed());
    SmartDashboard.putNumber("Gyro", m_driveTrain.getAngle());
    //SmartDashboard.putNumber("LiftPos", m_lift.talon.getSelectedSensorPosition());
    //SmartDashboard.putNumber("ArmPos", m_arm.getPos());
    SmartDashboard.putNumber("Resistance", m_driveTrain.getResistance());
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    
  }
}
