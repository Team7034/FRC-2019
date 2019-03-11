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

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.IRSensor;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class claw extends Subsystem {
  public WPI_TalonSRX talon;
  public WPI_TalonSRX talon2;
  public Solenoid claw;
  public IRSensor irsense;

  public double target_power = .5;

  @Override
  public void initDefaultCommand() {
    talon = new WPI_TalonSRX(RobotMap.intakeL);
    talon2 = new WPI_TalonSRX(RobotMap.intakeR);
    claw = new Solenoid(RobotMap.claw);

    talon.configOpenloopRamp(0);
    talon2.configOpenloopRamp(0);
    talon.setNeutralMode(NeutralMode.Brake);
    talon2.setNeutralMode(NeutralMode.Brake);

    irsense = new IRSensor(RobotMap.IRSensor);
  }
  
  //run the intake motors (arg changes direction)
  public void runIntake(double power){
    talon.set(ControlMode.PercentOutput, power);
    talon2.set(ControlMode.PercentOutput, power);
  }

  //adjust the target speed of the intake motors
  public void set_intake_speed(double speed){
    speed = Math.abs(speed); //make sure in and out don't get reversed
    if(speed > 1){           //make sure target speed doesnt exceed 100%
      speed = 1;
    }
    target_power = speed;
  }

  public void grab(Boolean state) {
    if (state) {
      claw.set(true);
    }
    else {
      claw.set(false);
    }
  }
}
