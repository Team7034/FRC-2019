package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;

public class Controller extends GenericHID {

	public Controller(final int port) {
		super(port);
	}
	
	//joysticks and triggers
	public final double getLX() {
		return getRawAxis(0);
	}
	
	public final double getLY() {
		return getRawAxis(1);
	}
	
	public final double getRX() {
		return getRawAxis(4);
	}
	
	public final double getRY() {
		return getRawAxis(5);
	}
	
	public final double getLT() {
		return getRawAxis(2);
	}
	
	public final double getRT() {
		return getRawAxis(3);
	}

	//buttons
	public final boolean getA() {
		return getRawButton(1);
	}
	
	public final boolean getB() {
		return getRawButton(2);
	}
	
	public final boolean getXB() {
		return getRawButton(3);
	}
	
	public final boolean getYB() {
		return getRawButton(4);
	}
	
	public final boolean getLB() {
		return getRawButton(5);
	}
	
	public final boolean getRB() {
		return getRawButton(6);
	}
	
	public final boolean getBack() {
		return getRawButton(7);
	}
	
	public final boolean getStart() {
		return getRawButton(8);
	}
	
	public final boolean getLSB() {
		return getRawButton(9);
	}
	
	public final boolean getRSB() {
		return getRawButton(10);
	}
	
	//dpad
	public final boolean getDPAD(String dir) {
		int val = getPOV(0);
		if (dir == "up" && val == 0) {
			return true;
		}
		else if(dir == "upr" && val == 45) {
			return true;
		}
		else if(dir == "right" && val == 90) {
			return true;
		}
		else if(dir == "downr" && val == 135) {
			return true;
		}
		else if(dir == "down" && val == 180) {
			return true;
		}
		else if(dir == "downl" && val == 225) {
			return true;
		}
		else if(dir == "left" && val == 270) {
			return true;
		}
		else if(dir == "upl" && val == 310) {
			return true;
		}
		else {
			return false;
		}
	}
	
	@Deprecated
	public double getX(Hand hand) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Deprecated
	public double getY(Hand hand) {
		// TODO Auto-generated method stub
		return 0;
	}
}
