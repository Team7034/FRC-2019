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
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  static Joystick stick = new Joystick(RobotMap.joystick); //LogitechX3D
  static Controller cont = new Controller(RobotMap.gamepad);
  static Joystick gunner = new Joystick(RobotMap.gunner);

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
  Button ball_grab_cont = new JoystickButton(cont, 7);
  Button arm_reverse = new JoystickButton(cont, 8);


  //button panel
  Button cargo_low = new JoystickButton(gunner, 1);
  Button cargo_mid = new JoystickButton(gunner, 2);
  Button cargo_high = new JoystickButton(gunner, 3);
  Button cargo_intake = new JoystickButton(gunner, 4);

  Button hatch_low = new JoystickButton(gunner, 5);
  Button hatch_mid = new JoystickButton(gunner, 6);
  Button hatch_high = new JoystickButton(gunner, 7);
  Button open_panel = new JoystickButton(gunner, 8);

  Button toggle_man = new JoystickButton(gunner, 9);
  Button blank = new JoystickButton(gunner, 10);
  Button up = new JoystickButton(gunner, 11);
  Button close_panel = new JoystickButton(gunner, 12);
  

  //Button gamepadB = new JoystickButton(gamepad, 2);
  //Button gamepadA = new JoystickButton(gamepad, 1);

  public OI() {
    joyTrigger.whenPressed(new shift());
    joyB2.whenPressed(new reverse());

    joyB3.whenPressed(new HABLiftFront(false));
    joyB4.whenPressed(new HABLiftRear(false));
    joyB5.whenPressed(new HABLiftUp());
    joyB6.whenPressed(new HABLiftUp());

    joyB11.whenPressed(new zeroSensors());
    joyB12.whenPressed(new pathFollower(new Path("-simple")));
    
    upright.whenPressed(new AutomaticArm(arm.state.get("rest")));
    forward.whenPressed(new AutomaticArm(arm.state.get("ballLow")));
    backward.whenPressed(new AutomaticArm(arm.state.get("ballHigh")));
    mid.whenPressed(new AutomaticArm(arm.state.get("ballMid")));
    ball_grab_cont.whenPressed(new AutomaticArm(arm.state.get("ballGrab")));
    arm_reverse.whenPressed(new ArmReverse());
    
    open.whenPressed(new grab(true));
    close.whenPressed(new grab(false));

    //button panel
    /*cargo_low.whenPressed(new AutomaticArm(arm.state.get("ballLow")));
    cargo_mid.whenPressed(new AutomaticArm(arm.state.get("ballMid")));
    cargo_high.whenPressed(new AutomaticArm(arm.state.get("ballHigh")));
    cargo_intake.whenPressed(new AutoGrab());

    hatch_low.whenPressed(new AutomaticArm(arm.state.get("hatchLow")));
    hatch_mid.whenPressed(new AutomaticArm(arm.state.get("hatchMid")));
    hatch_high.whenPressed(new AutomaticArm(arm.state.get("hatchHigh")));
    open_panel.whenPressed(new grab(true));
    
    //toggle_man
    toggle_man.toggleWhenPressed(new toggleManualArm());
    //blank
    up.whenPressed(new AutomaticArm(arm.state.get("rest")));
    close_panel.whenPressed(new grab(false));*/
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
