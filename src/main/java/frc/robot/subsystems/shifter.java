package frc.robot.subsystems;

import frc.robot.RobotMap;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class shifter extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	private DoubleSolenoid shifter;
	Compressor comp;

    public void initDefaultCommand() {
        shifter = new DoubleSolenoid(RobotMap.shifter[0], RobotMap.shifter[1]);
        comp = new Compressor(RobotMap.compressor);
    }
    
    public void compressorOn(boolean on) {
    	comp.setClosedLoopControl(on);
    }
    public void gear1() {
        shifter.set(DoubleSolenoid.Value.kForward);
    }
    public void gear2() {
        shifter.set(DoubleSolenoid.Value.kReverse);
    }
}

