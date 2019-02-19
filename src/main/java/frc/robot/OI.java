/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GamepadBase;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.HIDType;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
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
  static Controller gamepad = new Controller(RobotMap.gamepad);
  Button joyTrigger = new JoystickButton(stick, 1);
  Button joyB2 = new JoystickButton(stick, 2);
  Button joyB3 = new JoystickButton(stick, 3);
  Button joyB4 = new JoystickButton(stick, 4);
  Button joyB5 = new JoystickButton(stick, 5);
  Button joyB6 = new JoystickButton(stick, 6);
  Button joyB7 = new JoystickButton(stick, 7);
  Button joyB8 = new JoystickButton(stick, 8);

  Button joyB11 = new JoystickButton(stick, 11);
  Button joyB12 = new JoystickButton(stick, 12);
  

  //Button gamepadB = new JoystickButton(gamepad, 2);
  //Button gamepadA = new JoystickButton(gamepad, 1);

  public OI() {
    joyTrigger.whenPressed(new shift(false));
    joyTrigger.whenReleased(new shift(true));
    joyB2.whenPressed(new reverse());

    joyB3.whenPressed(new habLiftPistons(true, true));
    joyB4.whenPressed(new habLiftPistons(true, false));
    joyB5.whenPressed(new habLiftPistons(false, true));
    joyB6.whenPressed(new habLiftPistons(false, false));
    /*
    joyB3.whenPressed(new grab(true));
    joyB4.whenPressed(new grab(false));
    
    joyB7.whenReleased(new runIntake(0));
    joyB7.whenPressed(new runIntake(-1));
    joyB8.whenPressed(new runIntake(1));
    joyB8.whenReleased(new runIntake(0));
    */
    joyB11.whenPressed(new zeroSensors());
    joyB12.whenPressed(new pathFollower(SmartDashboard.getString("Testing Path", "simple")));
  }
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());

  public double getDriveY() {
    return stick.getY();
  }
  public double getDriveX() {
    return stick.getX();
  }
  public double getSlider() {
    return stick.getThrottle();
  }

  public double getRightX() {
    return gamepad.getRX();
  }
  public double getLeftY() {
    return gamepad.getLY();
  }
}
