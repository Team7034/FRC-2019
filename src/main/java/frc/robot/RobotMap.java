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
	public static final int driveL1 = 3;
  public static final int driveL2 = 0;
  public static final int driveL3 = 1;

  public static final int driveR1 = 4;
  public static final int driveR2 = 2;
  public static final int driveR3 = 5;

  //sparks
  public static final int neo1 = 0;

  //controllers
  public static final int joystick = 0;

  //pneumatics
  public static final int compressor = 0;
  public static final int[] shifterL = {0, 1};
  public static final int[] shifterR = {2, 3};
  
  
  
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
}
