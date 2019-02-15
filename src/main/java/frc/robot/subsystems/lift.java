/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;


/**
 * Add your docs here.
 */
public class lift extends Subsystem {
  public WPI_TalonSRX talon;
  public WPI_TalonSRX talon2;
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    talon = new WPI_TalonSRX(RobotMap.liftR); //main left motor
    talon2 = new WPI_TalonSRX(RobotMap.liftL);
    
    talon2.follow(talon);

    talon.setInverted(InvertType.InvertMotorOutput);
    talon2.setInverted(InvertType.InvertMotorOutput);

    talon.setSensorPhase(true);
    talon.config_kP(0, 0.1);
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void run(double power) {
    talon.set(power);
  }

  public void zero() {
    talon.setSelectedSensorPosition(0);
  }

  public int getPos() {
    return talon.getSelectedSensorPosition();
  }

  //Mechanical range is 2930000 encoder ticks
  public void setTarget(int target) {
    talon.set(ControlMode.Position, target);
  }
}
