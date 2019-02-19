package frc.robot.subsystems;

import frc.robot.RobotMap;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class shifter extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	private Solenoid shifter;
	Compressor comp;

    public void initDefaultCommand() {
        shifter = new Solenoid(RobotMap.shifter);
        comp = new Compressor(RobotMap.compressor);
    }
    
    public void compressorOn(boolean on) {
    	comp.setClosedLoopControl(on);
    }
    public void gear2() {
        shifter.set(true);
    }
    public void gear1() {
        shifter.set(false);
    }
}

