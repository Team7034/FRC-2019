/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutomaticArm extends CommandGroup {
  /**
   * Add your docs here.
   */
  public AutomaticArm(int target) {
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.
    addSequential(new MoveArm(target));
    addSequential(new MoveElevator(target));

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.

    /* plan
     * receive target
     * does the arm have to pass through?
     * if yes, lower elevator all the way down
     * once elevator is all the way down, move arm though
     * once arm is out of "dead zone," move elevator
     * 
     * if no, move elevator to height, move arm at same time
     * 
     * if arm is upright, move arm to position
     * once out of "dead zone," move elevator
     * 
     * -----POSITION SYSTEM-----
     * -want to only specify level, not direction
     * -list of possible positions, use robot direction to figure out rest
    */

    //does arm need to go through dead zone?
    //if signs of cur and tar are opposite, yes it does
    /*if((currentPosition >= 0 && targetPosition <= 0) || (currentPosition <= 0 && targetPosition >= 0)){
      //go through dead zone
      //lower elevator all the way
      //when elevator is fully lowered, move arm
      //when arm is clear of dead zone, move elevator
    }else{
      //does not go through dead zone
      //set arm position to target
      //set elevator position to target
    }*/
  }
}
