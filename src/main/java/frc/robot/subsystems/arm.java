/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;

/**
 * Add your docs here.
 */
public class arm extends Subsystem {
  public CANSparkMax arm;
  public CANSparkMax arm2;
  public WPI_TalonSRX lift;
  public WPI_TalonSRX lift2;


  CANPIDController armPID;
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    arm = new CANSparkMax(RobotMap.armL, CANSparkMaxLowLevel.MotorType.kBrushless);
    arm2 = new CANSparkMax(RobotMap.armR, CANSparkMaxLowLevel.MotorType.kBrushless);
    
    //followers
    arm2.follow(arm, true);


    armPID = arm.getPIDController();

    armPID.setP(1);
    armPID.setOutputRange(-.15, .15);

    arm.set(0);

    lift = new WPI_TalonSRX(RobotMap.liftR); //main left motor
    lift2 = new WPI_TalonSRX(RobotMap.liftL);
    
    lift2.follow(lift);

    lift.setInverted(InvertType.InvertMotorOutput);
    lift2.setInverted(InvertType.InvertMotorOutput);

    lift.setSensorPhase(true);
    lift.config_kP(0, 0.1);

    
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void runLift(double power) {
    lift.set(power);
  }

  public void zeroLiftEnc() {
    lift.setSelectedSensorPosition(0);
  }

  public int getLiftPos() {
    return lift.getSelectedSensorPosition();
  }

  //Mechanical range is 2930000 encoder ticks
  public void setLiftTarget(int target) {
    lift.set(ControlMode.Position, target);
  }

  public void runArm(double speed) {
    arm.set(speed);
  }

  public double getArmPos() {
    return arm.getEncoder().getPosition();
  }

  //Mechanical range is 39 revolutions (encoder ticks)
  public void setArmTarget(double target) {
    armPID.setReference(target, ControlType.kPosition);
  }
}
