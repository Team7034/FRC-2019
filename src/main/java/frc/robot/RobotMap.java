/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  //drive subsystem
  public static int driveL1 = 0;
  public static int driveL2 = 1;
  public static int driveL3 = 2;
  public static int driveR1 = 3;
  public static int driveR2 = 4;
  public static int driveR3 = 5;
  public static int shifter = 4;
  /*public static int shifterL1 = 0;
  public static int shifterL2 = 1;
  public static int shifterR1 = 6;
  public static int shifterR2 = 7;*/

  //arm subsystem
  public static int elevator1 = 8;
  public static int elevator2 = 9;
  public static int arm1 = 1; //arm motors are SparkMax, not Talon
  public static int arm2 = 2;

  //intake subsystem
  public static int intake1 = 6;
  public static int intake2 = 7;
  public static int grabber = 5; //grabbers are solenoid channels
  public static int IRSensor = 0;

  //LED subsytem
  public static int LED1 = 0;
  public static int LED2 = 1;

  //oi
  public static int joystick = 0;
  public static int gamepad = 1;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
}
