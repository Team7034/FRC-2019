/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Code for running the intake of the robot
 */
public class Intake extends Subsystem {
  //intake motors
  WPI_TalonSRX motor1;
  WPI_TalonSRX motor2;

  //arm solenoid
  //DoubleSolenoid grabber;
  Solenoid grabber;
  Compressor comp;

  //target power for the intake
  private double target_power = 1;

  //constructor
  public Intake(){
    //create and set up arm motors
    motor1 = new WPI_TalonSRX(RobotMap.intake1);
    motor2 = new WPI_TalonSRX(RobotMap.intake2);
    motor1.configOpenloopRamp(0);
    motor2.configOpenloopRamp(0);
    motor1.setNeutralMode(NeutralMode.Brake);
    motor2.setNeutralMode(NeutralMode.Brake);
    motor2.setInverted(false);

    //create arm solenoid
    //grabber = new DoubleSolenoid(RobotMap.grabber1, RobotMap.grabber2);
    grabber = new Solenoid(RobotMap.grabber);
    comp = new Compressor();
    comp.stop();
  }

  //open the grabber
  public void open(){
    //grabber.set(DoubleSolenoid.Value.kForward);
    grabber.set(true);
  }

  //close the grabber
  public void close(){
    //grabber.set(DoubleSolenoid.Value.kReverse);
    grabber.set(false);
  }

  //run the intake motors (arg changes direction)
  public void run_intake(boolean in){
    if(in){ //if going in, +
      motor1.set(ControlMode.PercentOutput, target_power);
      motor2.set(ControlMode.PercentOutput, target_power);
    }else{  //if going out, -
      motor1.set(ControlMode.PercentOutput, -target_power);
      motor2.set(ControlMode.PercentOutput, -target_power);
    }
  }

  //stop the intake motors
  public void stop_intake(){
    motor1.set(0);
    motor2.set(0);
  }

  //adjust the target speed of the intake motors
  public void set_intake_speed(double speed){
    speed = Math.abs(speed); //make sure in and out don't get reversed
    if(speed > 1){           //make sure target speed doesnt exceed 100%
      speed = 1;
    }
    target_power = speed;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
