/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

<<<<<<< HEAD
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
=======
import edu.wpi.first.wpilibj.TimedRobot;
>>>>>>> 473299e12e5f0e5777e11e8a92a088de9d1cb5f8
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
<<<<<<< HEAD
import frc.robot.commands.Drive;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.AutomaticArm;
import frc.robot.commands.ManualArm;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
=======
import frc.robot.subsystems.driveTrain;
import frc.robot.subsystems.pneumatics;
import frc.robot.subsystems.arm;
import frc.robot.commands.drive;
import frc.robot.commands.mainAuto;
import frc.robot.commands.moveArm;
>>>>>>> 473299e12e5f0e5777e11e8a92a088de9d1cb5f8

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
<<<<<<< HEAD
  public static ExampleSubsystem m_subsystem = new ExampleSubsystem();
  public static Arm m_arm = new Arm();
  public static Intake m_intake = new Intake();
  public static DriveTrain m_drive_train = new DriveTrain();
  public static OI m_oi;
  public static LED m_led;
  public static boolean forward = true;
  public static boolean arm_forward = true;
  public AHRS arm_gyro;
  private Alliance team = Alliance.Invalid;
=======
  public static OI m_oi;
  public static driveTrain m_driveTrain = new driveTrain();
  public static pneumatics m_pneumatics = new pneumatics();
  public static arm m_arm = new arm();
>>>>>>> 473299e12e5f0e5777e11e8a92a088de9d1cb5f8

  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
<<<<<<< HEAD
    team = m_ds.getAlliance();
    m_led = new LED(team);
    m_oi = new OI();
    m_chooser.setDefaultOption("Default Auto", new ExampleCommand());
    // chooser.addOption("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", m_chooser);
    //arm_gyro = new AHRS(Port.kUSB1);
=======
    m_oi = new OI();
    m_chooser.setDefaultOption("Default Auto", new mainAuto());
    //chooser.addOption("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", m_chooser);
>>>>>>> 473299e12e5f0e5777e11e8a92a088de9d1cb5f8
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
<<<<<<< HEAD
    if(team == Alliance.Invalid){
      team = m_ds.getAlliance();
      m_led.updateAlliance(team);
    }
=======
>>>>>>> 473299e12e5f0e5777e11e8a92a088de9d1cb5f8
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
<<<<<<< HEAD
    
=======
    Scheduler.getInstance().add(new drive());
    Scheduler.getInstance().add(new moveArm());
>>>>>>> 473299e12e5f0e5777e11e8a92a088de9d1cb5f8
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
<<<<<<< HEAD
    Scheduler.getInstance().add(new ManualArm());
    //Scheduler.getInstance().add(new Drive());
    /*if(Robot.m_oi.cont.getDPAD("left")){
      Scheduler.getInstance().add(new AutomaticArm(Arm.state.get("hatchLow")));
    }else if(Robot.m_oi.cont.getDPAD("right")){
      Scheduler.getInstance().add(new AutomaticArm(Arm.state.get("hatchMid")));
    }*/

    Scheduler.getInstance().run();
    /*SmartDashboard.putBoolean("forward", forward);
    if(m_oi.cont.getDPAD("up")){
      m_intake.run_intake(true);
    }else if(m_oi.cont.getDPAD("down")){
      m_intake.run_intake(false);
    }else{
      m_intake.stop_intake();
    }*/
=======
    Scheduler.getInstance().run();
    SmartDashboard.putNumber("PositionL", m_driveTrain.talonL.getSelectedSensorPosition());
    SmartDashboard.putNumber("PositionR", m_driveTrain.talonR.getSelectedSensorPosition());
    SmartDashboard.putNumber("VelocityL", m_driveTrain.talonL.getSelectedSensorVelocity());
    SmartDashboard.putNumber("VelocityR", m_driveTrain.talonR.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Gyro", m_driveTrain.getAngle());
>>>>>>> 473299e12e5f0e5777e11e8a92a088de9d1cb5f8
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
