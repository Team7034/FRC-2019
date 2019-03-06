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
  //talonSRX
	public static final int driveL1 = 0;
  public static final int driveL2 = 1;
  public static final int driveL3 = 2;

  public static final int driveR1 = 3;
  public static final int driveR2 = 4;
  public static final int driveR3 = 5;

  public static final int intakeL = 6;
  public static final int intakeR = 7;
  
  public static final int liftL = 8;
  public static final int liftR = 9;

  public static final int habLift = 10;

  //SparkMaxs
  public static final int armL = 1;
  public static final int armR = 2;

  //controllers
  public static final int joystick = 0;
  public static final int gamepad = 1;

  //pneumatics
  public static final int compressor = 0;
  public static final int[] habLiftRear = {0, 1};
  public static final int[] habLiftFront = {2, 3};
  public static final int shifter = 4;
  public static final int claw = 5;

  //sensors
  public static int IRSensor = 0;

  //LED subsytem
  public static int LED1 = 0;
  public static int LED2 = 1;
}
