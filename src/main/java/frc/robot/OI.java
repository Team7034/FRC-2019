/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.*;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  static Joystick stick = new Joystick(RobotMap.joystick); //LogitechX3D
  static Controller cont = new Controller(RobotMap.gamepad);
  Button joyTrigger = new JoystickButton(stick, 1);
  Button joyB2 = new JoystickButton(stick, 2);
  Button joyB3 = new JoystickButton(stick, 3);
  Button joyB4 = new JoystickButton(stick, 4);
  Button joyB5 = new JoystickButton(stick, 5);
  Button joyB6 = new JoystickButton(stick, 6);
  Button joyB7 = new JoystickButton(stick, 7);
  Button joyB8 = new JoystickButton(stick, 8);
  Button joyB9 = new JoystickButton(stick, 9);
  Button joyB10 = new JoystickButton(stick, 10);
  Button joyB11 = new JoystickButton(stick, 11);
  Button joyB12 = new JoystickButton(stick, 12);

  Button mid = new JoystickButton(cont, 1);
  Button backward = new JoystickButton(cont, 2);
  Button forward = new JoystickButton(cont, 3);
  Button upright = new JoystickButton(cont, 4);
  Button open = new JoystickButton(cont, 5);
  Button close = new JoystickButton(cont, 6);
  Button hatchHigh = new JoystickButton(cont, 7);
  Button arm_reverse = new JoystickButton(cont, 8);
  

  //Button gamepadB = new JoystickButton(gamepad, 2);
  //Button gamepadA = new JoystickButton(gamepad, 1);

  public OI() {
    joyTrigger.whenPressed(new Shift());
    joyB2.whenPressed(new Reverse());
    
    joyB9.whenPressed(new Grab(true));
    joyB9.whenReleased(new Grab(false));

    joyB11.whenPressed(new ZeroSensors());
    joyB12.whenPressed(new PathFollower("-simple"));
    
    /*upright.whenPressed(new AutomaticArm(Arm.state.get("rest")));
    forward.whenPressed(new AutomaticArm(Arm.state.get("ballLow")));
    backward.whenPressed(new AutomaticArm(Arm.state.get("ballHigh")));
    mid.whenPressed(new AutomaticArm(Arm.state.get("ballMid")));
    hatchHigh.whenPressed(new AutomaticArm(Arm.state.get("hatchHigh")));
    arm_reverse.whenPressed(new ArmReverse());
    */
    open.whenPressed(new Grab(false));
    close.whenPressed(new Grab(true));
  }
  

  public double getDriveY() {
    return stick.getY();
  }
  public double getDriveX() {
    return stick.getX();
  }
  public double getSlider() {
    return stick.getThrottle();
  }

  public double getArm(){
    if(Math.abs(cont.getRY()) > .05){
      return cont.getRY();
    }
    return 0;
  }

  public double getElevator(){
    if(Math.abs(cont.getLY()) > .05){
      return cont.getLY();
    }
    return 0;
  }
}
