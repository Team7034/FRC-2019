/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Map;
import java.util.Collections;
import java.util.HashMap;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.ArmPIDController;
import frc.robot.PIDDebug;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.SparkMaxEncoder;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

/**
 * Methods that control the elevator, the arm, and their interactions
 */
public class Arm extends Subsystem {
  //motor variables
  public WPI_TalonSRX elevator1;
  private WPI_TalonSRX elevator2;
  private CANSparkMax arm1;
  private CANSparkMax arm2;

  //list of possible arm states
  public static Map<String, Integer> state;

  //pid targets for various arm set heights
  //upright
  //hatch low
  //hatch mid
  //hatch high
  //ball low
  //ball mid
  //ball high
  private double armPosition[] = {0, 12, 10, 4.5, 12, 8, 3.5};

  //pid targets for various elevator set heights
  //bottom
  //hatch low
  //hatch mid
  //hatch high
  //ball low
  //ball mid
  //ball high
  private int elevatorPosition[] = {0, 10000, 30000, 34000, 18000, 28000, 34000};

  //PID controllers
  public ArmPIDController controller;

  //tolerance thresholds for arm
  private double arm_threshold = 2;

  public static double arm_target = 0;

  public Arm(){
    //create elevator motors and set up PID
    elevator1 = new WPI_TalonSRX(RobotMap.elevator1);
    elevator2 = new WPI_TalonSRX(RobotMap.elevator2);
    elevator2.follow(elevator1);
    elevator2.setInverted(false);
    elevator1.setSensorPhase(false);
    elevator1.configClosedLoopPeakOutput(0, 1);
    elevator1.setSelectedSensorPosition(0);
    elevator1.setNeutralMode(NeutralMode.Brake);
    elevator2.setNeutralMode(NeutralMode.Brake);
    //elevator1.configForwardSoftLimitThreshold(2550000);
    //elevator1.configReverseSoftLimitThreshold(0);
    elevator1.config_kP(0, 0.75);
    elevator1.config_kI(0, 0);
    elevator1.config_kD(0, 0);
    elevator1.config_kF(0, 0);

    //create arm motors and set up PID
    arm1 = new CANSparkMax(RobotMap.arm1, CANSparkMaxLowLevel.MotorType.kBrushless);
    arm2 = new CANSparkMax(RobotMap.arm2, CANSparkMaxLowLevel.MotorType.kBrushless);
    arm2.follow(arm1, true);
    //arm PID controller
    //0.1, 0, 0.13, 0.035
    controller = new ArmPIDController(0.1, 0, 0.16, 0.035, new SparkMaxEncoder(arm1.getEncoder()), arm1);
    controller.setOutputRange(-.25, .25); //max output power
    controller.enable();
    controller.setSetpoint(0);

    //used for tuning PID values
    //SmartDashboard.putData("PID Controller",controller);


    //intialize hashmap of states
    Map<String, Integer> tempListMap = new HashMap<String, Integer>();
    tempListMap.put("rest", 0);     //upright position
    tempListMap.put("intake", 1);
    tempListMap.put("hatchLow", 1);
    tempListMap.put("hatchMid", 2);
    tempListMap.put("hatchHigh", 3);
    tempListMap.put("ballLow", 4);
    tempListMap.put("ballMid", 5);
    tempListMap.put("ballHigh", 6);
    state = Collections.unmodifiableMap(tempListMap);
  }

  /**
   * Moves the arm to a target position.
   * @param target_level  the target level, from Arm.LiftLevel
   * @return              has the arm reached the target positon?
   */
  public boolean setArm(int target_level){
    arm_target = armPosition[target_level];
    if(Robot.arm_forward){
      arm_target *= -1;
    }

    //check to see if arm needs to pass through
    //if signs do not match (one + and one -) it must pass through. or if target == 0
    if(((arm_target<0) != (get_arm_pos()<0)) || arm_target == 0){
      //arm is passing through
      //check if elevator is ready for passthrough
      if(!ele_ready_passthrough()){
        //if not ready for passthrough, move arm to closest safe spot without passthrough
        //lower ele to bottom
        auto_ele(0);
        //if target is less than 0 and it needs pass through, arm must be positive
        //set arm to largest positive value
        if(arm_target < 0){
          controller.setSetpoint(armPosition[3]);
        }else if (arm_target > 0){
          controller.setSetpoint(-1*armPosition[3]);
        }
        return false; //false means command is not done, currently waiting on arm to go down
      }//if it is ready, it will simply continue and run the arm
    }
    else{
      controller.setSetpoint(arm_target);
      return true;
    }

    //update target
    controller.setSetpoint(arm_target);
    

    //check to see if arm has reached target (within threshhold of error)
    if(Math.abs(arm1.getEncoder().getPosition() - arm_target) < arm_threshold){
      return true;  //arm is in correct position
    }
    //or, if arm enters dead zone (passthrough), end, and ele will start when arm is clear
    return !arm_ready_raise();
    //arm has not reached correct position
  }
  
  public boolean auto_ele(int target){
    int ele_tar = elevatorPosition[target];
    if(arm_ready_raise()){
      elevator1.set(ControlMode.Position, ele_tar);
    }else{
      elevator1.set(ControlMode.Position, 0);
    }
    if(Math.abs(get_ele_pos()) < 50){
      return true;
    }
    return false;
  }

  public void manual_ele(double tar){
    elevator1.set(ControlMode.PercentOutput, tar);
  }

  public void manual_arm(double tar){
    arm_target += tar;
    controller.setSetpoint(arm_target);
  }

  public double get_arm_pos(){
    return arm1.getEncoder().getPosition();
  }

  public int get_ele_pos(){
    return elevator1.getSelectedSensorPosition();
  }

  private boolean ele_ready_passthrough(){
    return (get_ele_pos() < 2000);
  }

  private boolean arm_ready_raise(){
    //check to see if arm is within +/- 3 ticks of center (arm would hit if ele raised)
    if(Math.abs(get_arm_pos()) < 3){
      return false; //arm not ready for ele to be raised
    }
    return true;  //arm is ready for ele to be raised
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
