/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotMap;

import java.awt.Color;

import com.mach.LightDrive.LightDrivePWM;

/**
 * Add your docs here.
 */
public class LED extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  LightDrivePWM controller;
  Alliance team;

  public LED(Alliance team_color){
    controller = new LightDrivePWM(new Servo(RobotMap.LED1), new Servo(RobotMap.LED2));
    team = team_color;
    on(0);
    on(1);
    controller.Update();
  }

  public void update(boolean forward){
    if(forward){
      //set forward LEDS to team color and rear to off
      on(1);
      off(2);
      //front leds
      on(3);
      off(4);
    }else{
      //set rear LEDS to team color and forward to off
      off(1);
      on(2);
      //front leds
      on(4);
      off(3);
    }
    controller.Update();
  }

  private void on(int channel){
    //take channel (1-4) and set it to alliance's color (white for none)
    if(team == Alliance.Red){
      controller.SetColor(channel, Color.red);
    }else if(team == Alliance.Blue){
      controller.SetColor(channel, Color.green);//RBG not RGB, thus feed "fake" color for blue
    }else{  //Alliance.Invalid
      controller.SetColor(channel, Color.white);
    }
  }

  private void off(int channel){
    controller.SetColor(channel, Color.black);
  }

  public void updateAlliance(Alliance alli){
    team = alli;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
