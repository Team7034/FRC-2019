/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

//import com.sun.org.apache.xerces.internal.jaxp.DocumentBuilderImpl;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

import com.kauailabs.navx.frc.AHRS;
//import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.I2C;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import frc.robot.Robot;

public class turnToAngle extends Command {

double angle;
//double distance;
double angleToTurn;
double rotateToAngleRate;
Robot robot;

AHRS ahrs;
PIDController turnController;

static final double kP = 0.03;
static final double kI = 0.00;
static final double kD = 0.00;
static final double kF = 0.00;

private boolean turned = false;

boolean rotateToAngle;

static final double kToleranceDegrees = 2.0f;
  

  public turnToAngle(double a, Robot r) {

    robot = r;
    angle = (int) a;
    //distance = d;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);

    try {

      ahrs=  new AHRS(I2C.Port.kMXP);

    }
    catch (RuntimeException ex) {
      System.out.println("error connecting navx MXP" + ex.getMessage());
    }
    turnController = new PIDController(kP, kI, kD, kF, ahrs, robot.left_motors); //controls left motors

    turnController.setInputRange(-180.0f, 180.0f);
    turnController.setOutputRange(-1.0,1.0);
    turnController.setAbsoluteTolerance(kToleranceDegrees);
    turnController.setContinuous(true);


  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    
    angleToTurn = angle/2;
    angleToTurn += (double) 0.0f;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    double startAngle = ahrs.getAngle();

    if (angleToTurn < 0)
    {
      while (angleToTurn > startAngle)
      {
        robot.left_motors.set(0.5);
        robot.right_motors.set(-0.5);
      }
    }
    else if (angleToTurn > 0)
    {
       while (angleToTurn < startAngle)
      {
        robot.left_motors.set(-0.5);
        robot.right_motors.set(0.5);
      }
    }

    //turn the robot until you get to angleToTurn (NavX)
    //go forward until you get to the white line
    //line follow for like a bit //scratch that, no line following allowed - Parker

    /*

    turnController.setSetpoint(angleToTurn);
    rotateToAngle = true;

    //double currentRotationRate;

    if (rotateToAngle == true)
    {
      turnController.enable();
      robot.left_motors.set(rotateToAngleRate);
      robot.right_motors.set(-rotateToAngleRate);

      if (turnController.onTarget())
      {
        turnController.disable();
        turned = true;
      }
    } //how to access the instance of the robot we create in a command // done
*/
  }

  // Make this return true when this Command no longer needs to run execute() //called periodically
  @Override
  protected boolean isFinished() {
    return turned;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    robot.right_motors.set(0);
    robot.left_motors.set(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    robot.right_motors.set(0);
    robot.left_motors.set(0);
  }

  public void pidWrite(double output)
  {
    rotateToAngleRate = output;
  }
}
