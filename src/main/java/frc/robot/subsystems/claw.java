/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class claw extends Subsystem {
  public WPI_TalonSRX talon;
  public WPI_TalonSRX talon2;
  public Solenoid claw;

  @Override
  public void initDefaultCommand() {
    talon = new WPI_TalonSRX(RobotMap.intakeL);
    talon2 = new WPI_TalonSRX(RobotMap.intakeR);
    claw = new Solenoid(RobotMap.claw);
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  
  public void runIntake(double speed) {
    talon.set(-speed);
    talon2.set(speed);
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
