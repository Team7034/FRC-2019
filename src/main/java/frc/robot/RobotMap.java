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
  public static final int habLift2 = 11;

  //SparkMaxs
  public static final int armL = 0;
  public static final int armR = 1;


  //controllers
  public static final int joystick = 0;
  public static final int gamepad = 1;

  //pneumatics
  public static final int compressor = 0;
  public static final int[] habLiftBack = {0, 1};
  public static final int[] habLiftFront = {2, 3};
  public static final int shifter = 4;
  public static final int claw = 5;
  

  
  
  
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
}
