/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
  public static arm m_arm = new arm();
  public static driveTrain m_driveTrain = new driveTrain();
  public static claw m_claw = new claw();
  public static habLift m_habLift = new habLift();
  public static LED m_led;
  public static boolean forward = true;

  public static DashboardListener listener = new DashboardListener();
  private Alliance team = Alliance.Invalid;
  PowerDistributionPanel pdp;

  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    team = m_ds.getAlliance();
    m_led = new LED(team);
    m_oi = new OI();
    m_chooser.setDefaultOption("Default Auto", new RocketFinder());
    //chooser.addOption("My Auto", new MyAutoCommand());
    pdp = new PowerDistributionPanel();
    pdp.clearStickyFaults();
    SmartDashboard.putData("Auto mode", m_chooser);
    SmartDashboard.putString("Testing Path", "simple");
    //new LaunchPadListener();
    SmartDashboard.putBoolean("automatic", false);
    SmartDashboard.putBoolean("highGear", true);
    SmartDashboard.putBoolean("forward", true);
    SmartDashboard.putNumber("Path P", 5);
    SmartDashboard.putNumber("Path I", 0);
    SmartDashboard.putNumber("Path D", 0);
    SmartDashboard.putNumber("Path A", 0);
    SmartDashboard.putNumber("Path G", 0.8);
    SmartDashboard.putNumber("listenerTest", 0);

    listener.run();
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
    if(team == Alliance.Invalid){
      team = m_ds.getAlliance();
      m_led.updateAlliance(team);
    }
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
    m_driveTrain.setLocation(0, 0);

    //Scheduler.getInstance().add(new moveArm());
    //Scheduler.getInstance().add(new moveLift());
    //Scheduler.getInstance().add(new AutoShifter());
    SmartDashboard.putData(new autoShifter());
    //SmartDashboard.putData(new PathToPoint(2, 0, 0));
    //SmartDashboard.putNumberArray("pos_array", new double[]{0, 0});
    SmartDashboard.putNumber("x_target", 0);
    SmartDashboard.putNumber("y_target", 0);
    SmartDashboard.putData(new ToggleCompressor());
    //Robot.m_led.update(false);
    //Scheduler.getInstance().add(new reverse());
    m_driveTrain.auto = false;
    if(team == Alliance.Invalid){
      team = m_ds.getAlliance();
      m_led.updateAlliance(team);
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    if (Math.abs(m_oi.getDriveX()) > .05 || Math.abs(m_oi.getDriveY()) > .05) {
      m_driveTrain.auto = false;
    }
    if(!m_driveTrain.auto) {
      Scheduler.getInstance().add(new drive());
    }

    Scheduler.getInstance().add(new ManualArm());
    
    if (OI.cont.getDPAD("up")) {
      m_claw.runIntake(m_claw.target_power);
    }
    else if (OI.cont.getDPAD("down")) {
      m_claw.runIntake(-m_claw.target_power);
    }
    else {
      m_claw.runIntake(0);
    }

    Scheduler.getInstance().run();

    SmartDashboard.putNumber("ele enc", m_arm.get_ele_pos());
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_driveTrain.setLocation(0, 0);

    SmartDashboard.putData(new ToggleCompressor());
    m_driveTrain.auto = false;
    if(team == Alliance.Invalid){
      team = m_ds.getAlliance();
      m_led.updateAlliance(team);
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    if (Math.abs(m_oi.getDriveX()) > .05 || Math.abs(m_oi.getDriveY()) > .05) {
      m_driveTrain.auto = false;
    }
    if(!m_driveTrain.auto) {
      Scheduler.getInstance().add(new drive());
    }
    
    if (OI.cont.getDPAD("up")) {
      m_claw.runIntake(m_claw.target_power);
    }
    else if (OI.cont.getDPAD("down")) {
      m_claw.runIntake(-m_claw.target_power);
    }
    else {
      m_claw.runIntake(0);
    }

    SmartDashboard.putNumber("ele enc", m_arm.get_ele_pos());

    if(OI.cont.getRSB()){
      m_arm.arm_forward = true;
      m_arm.controller.setOutputRange(-.1, .1);
      m_arm.controller.setD(.13);
      double target = 0;
      double roll = m_driveTrain.gyro.getRoll();
      if(roll < 0){
        target = .3* roll;
      } else if(roll > 0 && roll < 1.5){
        target = -.25;
      } else if(roll > 1.5){
        target = .2* roll;
      }    
      if(target > 0){
        target = 0;
      }else if(target < -1){
          target = -1;
      }
      m_arm.controller.setSetpoint(target);
    }else{
      m_arm.controller.setOutputRange(-.25, .25);
      m_arm.controller.setD(.16);
      //m_arm.controller.setSetpoint(0);
      //Scheduler.getInstance().add(new ManualArm());
    }

    Scheduler.getInstance().run();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    
  }
}
