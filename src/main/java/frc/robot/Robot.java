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
  public static boolean arm_forward = true;

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
    
    //Scheduler.getInstance().add(new moveArm());
    //Scheduler.getInstance().add(new moveLift());
    //Scheduler.getInstance().add(new AutoShifter());
    SmartDashboard.putData(new AutoShifter());
    //SmartDashboard.putData(new PathToPoint(2, 0, 0));
    //SmartDashboard.putNumberArray("pos_array", new double[]{0, 0});
    SmartDashboard.putNumber("xTarget", 0);
    SmartDashboard.putNumber("yTarget", 0);
    SmartDashboard.putData(new ToggleCompressor());
    
    listener.run();
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
      Scheduler.getInstance().add(new Drive());
    }
    
    double angle = m_driveTrain.gyro.getYaw();
    SmartDashboard.putNumber("teleop angle", angle);

    /*
    if(OI.cont.getDPAD("left")){
      Scheduler.getInstance().add(new HABLiftRear(true));
      //Scheduler.getInstance().add(new AutomaticArm(Arm.state.get("hatchLow")));
    }else if(OI.cont.getDPAD("right")){
      Scheduler.getInstance().add(new HABLiftRear(false));
      //Scheduler.getInstance().add(new AutomaticArm(Arm.state.get("hatchMid")));
    }
    */

    double target = 0;
    double roll = m_driveTrain.gyro.getRoll();
    if(roll < 0){
      target = .4* roll;
    } else if(roll > 0 && roll < 1.5){
      target = -.25;
    } else if(roll > 1.5){
      target = .2* roll;
    }
    
    if(target > 1){
      target = 1;
    }else if(target < -1){
      target = -1;
    }
    //m_arm.arm_target = target;
    //m_arm.manual_arm(target);

    //m_arm.controller.setSetpoint(target);
    
    m_arm.controller.setOutputRange(-.1, .1);
    m_arm.controller.setP(0.1);
    m_arm.controller.setD(.13);
    m_arm.controller.setF(0.035);
    SmartDashboard.putNumber("AutoBal Tar", target);
    SmartDashboard.putNumber("Arm Enc", m_arm.get_arm_pos());

    //SmartDashboard.putBoolean("forward", forward);
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

    SmartDashboard.putNumber("navx pitch", m_driveTrain.gyro.getPitch());
    SmartDashboard.putNumber("navx?", m_driveTrain.gyro.getRawGyroX());
    SmartDashboard.putNumber("navx roll", m_driveTrain.gyro.getRoll());
    SmartDashboard.putNumber("navx yaw", m_driveTrain.gyro.getYaw());
    SmartDashboard.putNumber("PositionL", m_driveTrain.getEncPosL());
    SmartDashboard.putNumber("PositionR", m_driveTrain.getEncPosR());
    SmartDashboard.putNumber("Speed", m_driveTrain.getSpeed());
    SmartDashboard.putNumber("Resistance", m_driveTrain.getResistance());
    SmartDashboard.putBoolean("Auto", m_driveTrain.auto);
    SmartDashboard.putNumber("Robot x", m_driveTrain.xPos);
    SmartDashboard.putNumber("Robot y", m_driveTrain.yPos);
    SmartDashboard.putBoolean("High Gear", m_driveTrain.inHighGear());
    SmartDashboard.putBoolean("forward", !m_driveTrain.reversed);
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    
  }
}
