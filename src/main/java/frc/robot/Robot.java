/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.SerialPort;
//import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.*;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.HABLift;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;

public class Robot extends TimedRobot{
  public static Arm m_arm = new Arm();
  public static Intake m_intake = new Intake();
  public static DriveTrain m_drive_train = new DriveTrain();
  public static HABLift m_lift = new HABLift();
  public static OI m_oi;
  public static LED m_led;
  public static boolean forward = true;
  public static boolean arm_forward = true;
  public AHRS navx;
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
    //m_chooser.setDefaultOption("Default Auto", new ExampleCommand());
    // chooser.addOption("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", m_chooser);
    try {
			navx = new AHRS(Port.kMXP);
		} catch(RuntimeException e) {
      DriverStation.reportError("Error Instantiating the NavX Micro: " + e.getMessage(), true);
    }
    pdp = new PowerDistributionPanel();
    pdp.clearStickyFaults();
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
    // Scheduler.getInstance().add(new drive());
    // Scheduler.getInstance().add(new moveArm());
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    //Scheduler.getInstance().add(new ManualArm());
    //SmartDashboard.putBoolean("HaveBall", m_intake.irsense.read());
    Scheduler.getInstance().add(new Drive());
    if(Robot.m_oi.cont.getDPAD("left")){
      m_lift.lift.set(DoubleSolenoid.Value.kForward);
      //Scheduler.getInstance().add(new AutomaticArm(Arm.state.get("hatchLow")));
    }else if(Robot.m_oi.cont.getDPAD("right")){
      m_lift.lift.set(DoubleSolenoid.Value.kReverse);
      //Scheduler.getInstance().add(new AutomaticArm(Arm.state.get("hatchMid")));
    }



    SmartDashboard.putNumber("navx pitch", navx.getPitch());
    SmartDashboard.putNumber("navx?", navx.getRawGyroX());
    SmartDashboard.putNumber("navx roll", navx.getRoll());
    SmartDashboard.putNumber("navx yaw", navx.getYaw());
    

    double target = 0;
    if(navx.getRoll() < 0){
      target = .4* navx.getRoll();
    }else if(navx.getRoll() > 0 && navx.getRoll() > 1.5){
      target = -.25;
    }else if(navx.getRoll() > 1.5){
      target = .2* navx.getRoll();
    }
    
    if(target > 1){
      target = 1;
    }else if(target < -1){
      target = -1;
    }
    //m_arm.arm_target = target;
    //m_arm.manual_arm(target);
    m_arm.controller.setSetpoint(target);
    
    m_arm.controller.setOutputRange(-.1, .1);
    m_arm.controller.setP(0.1);
    m_arm.controller.setD(.13);
    m_arm.controller.setF(0.035);
    SmartDashboard.putNumber("autobal tar", target);
    SmartDashboard.putNumber("armEnc", m_arm.get_arm_pos());





    Scheduler.getInstance().run();
    //SmartDashboard.putBoolean("forward", forward);
    if(m_oi.cont.getDPAD("up")){
      m_intake.run_intake(true);
    }else if(m_oi.cont.getDPAD("down")){
      m_intake.run_intake(false);
    }else{
      m_intake.stop_intake();
    }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
