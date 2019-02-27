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
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.IntakeOpen;
import frc.robot.commands.ArmReverse;
import frc.robot.commands.IntakeClose;
import frc.robot.commands.AutomaticArm;
import frc.robot.commands.Shift;
import frc.robot.commands.Reverse;
import frc.robot.subsystems.Arm;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
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

  public static Controller cont = new Controller(RobotMap.gamepad);
  static Joystick stick = new Joystick(RobotMap.joystick);

  Button upright = new JoystickButton(cont, 4);
  Button forward = new JoystickButton(cont, 3);
  Button backward = new JoystickButton(cont, 2);
  Button mid = new JoystickButton(cont, 1);
  Button hatchHigh = new JoystickButton(cont, 7);

  Button arm_reverse = new JoystickButton(cont, 8);
  Button reverse = new JoystickButton(stick, 2);

  Button open = new JoystickButton(cont, 5);
  Button close = new JoystickButton(cont, 6);


  Button shift = new JoystickButton(stick, 1);

  public OI(){
    /*upright.whenPressed(new AutomaticArm(Arm.state.get("rest")));
    forward.whenPressed(new AutomaticArm(Arm.state.get("ballLow")));
    backward.whenPressed(new AutomaticArm(Arm.state.get("ballHigh")));
    mid.whenPressed(new AutomaticArm(Arm.state.get("ballMid")));
    hatchHigh.whenPressed(new AutomaticArm(Arm.state.get("hatchHigh")));
    arm_reverse.whenPressed(new ArmReverse());
    reverse.whenPressed(new Reverse());
    shift.whenPressed(new Shift(true));
    shift.whenReleased(new Shift(false));*/
    open.whenPressed(new IntakeOpen());
    close.whenPressed(new IntakeClose());
    arm_reverse.whenPressed(new ArmReverse());
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

  public double getDriveY(){
    return stick.getY();
  }

  public double getDriveX(){
    return stick.getX();
  }

}
