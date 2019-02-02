/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.ConfigParameter;

/**
 * Add your docs here.
 */
public class arm extends Subsystem {
  public CANSparkMax neo;
  public CANSparkMax neo2;
  public WPI_TalonSRX talon;
  public WPI_TalonSRX talon2;


  CANPIDController controller;
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    neo = new CANSparkMax(RobotMap.armL, CANSparkMaxLowLevel.MotorType.kBrushless);
    neo2 = new CANSparkMax(RobotMap.armR, CANSparkMaxLowLevel.MotorType.kBrushless);
    talon = new WPI_TalonSRX(RobotMap.intakeL);
    talon2 = new WPI_TalonSRX(RobotMap.intakeR);

    //followers
    neo2.follow(neo, true);
    talon2.follow(talon);
    talon.setInverted(true);

    controller = neo.getPIDController();

    controller.setP(1);
    controller.setOutputRange(-.1, .1);

    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void run(double speed) {
    neo.set(speed);
  }

  public double getPos() {
    return neo.getEncoder().getPosition();
  }

  public void setTarget(double target) {
    controller.setReference(target, ControlType.kPosition);
  }

  public void runIntake(double speed) {
    talon.set(speed);
  }
}
