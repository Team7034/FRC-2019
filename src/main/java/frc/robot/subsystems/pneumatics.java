package frc.robot.subsystems;

import frc.robot.RobotMap;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class pneumatics extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	private DoubleSolenoid leftShifter, rightShifter;
	Compressor comp;

    public void initDefaultCommand() {
        leftShifter = new DoubleSolenoid(RobotMap.shifterL[0], RobotMap.shifterL[1]);
        rightShifter = new DoubleSolenoid(RobotMap.shifterR[0], RobotMap.shifterR[1]);
        comp = new Compressor(RobotMap.compressor);
    }
    
    public void compressorOn(boolean on) {
    	comp.setClosedLoopControl(on);
    }
    public void extendShifter() {
        leftShifter.set(DoubleSolenoid.Value.kForward);
        rightShifter.set(DoubleSolenoid.Value.kForward);
    }
    public void retractShifter() {
        leftShifter.set(DoubleSolenoid.Value.kReverse);
        rightShifter.set(DoubleSolenoid.Value.kReverse);
    }
}

