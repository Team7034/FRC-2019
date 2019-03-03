/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class habLift extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private DoubleSolenoid rear;
  private DoubleSolenoid front;
  private WPI_TalonSRX talon;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    rear = new DoubleSolenoid(RobotMap.habLiftRear[0], RobotMap.habLiftRear[1]);
    front = new DoubleSolenoid(RobotMap.habLiftFront[0], RobotMap.habLiftFront[1]);
  }

  public void extendFront() {
    front.set(DoubleSolenoid.Value.kForward);
  }
  public void retractFront() {
    front.set(DoubleSolenoid.Value.kReverse);
  }
  public void extendRear() {
    rear.set(DoubleSolenoid.Value.kForward);
  }
  public void retractRear() {
    rear.set(DoubleSolenoid.Value.kReverse);
  }

  public void runHabLift(double value) {
    talon.set(value);
  }
}
