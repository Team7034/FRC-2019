/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

/**
 * Add your docs here.
 */
public class Path {

    private final double wheelbase_width = 0.646;
	private final double wheel_diameter = 0.104;
    private final double max_velocity = 2.5;
    private final double max_acceleration = 5;
    private final double max_jerk = 60;
    private final int TICKS_PER_REV = 18000;
    public static double kP = 5;
	public static double kI = 0;
	public static double kD = 0.1;
	public static double kA = 0;
	public static double kG = 0.8 * (-1.0/80.0);
    
    private boolean flip;
    private String pathName;
    private EncoderFollower left;
    private EncoderFollower right;
    private double dt = 0.05;
    private double offset = 0;

    public Path(String pathName) {
        kP = SmartDashboard.getNumber("Path P", kP);
		kI = SmartDashboard.getNumber("Path I", kI);
		kD = SmartDashboard.getNumber("Path D", kD);
        kA = SmartDashboard.getNumber("Path A", kA);
        kG = SmartDashboard.getNumber("Path G", kG);
        if (pathName.substring(0,1) == "+") {
			flip = true;
		}
		else if (pathName.substring(0,1) == "-"){
			flip = false;
        }
        String pathFileName = pathName.substring(1);

        //Generates left and right trajectories from csv files on the roboRIO
        //DOES NOT WORK WITH LATEST VERSION OF PATHFINDER, USE 2019.1.12
        Trajectory lTrajectory;
        Trajectory rTrajectory;
        if (!flip) {
            lTrajectory = PathfinderFRC.getTrajectory("output/" + pathFileName + ".left");
		    rTrajectory = PathfinderFRC.getTrajectory("output/" + pathFileName + ".right");
        }
        else {
            lTrajectory = PathfinderFRC.getTrajectory("output/" + pathFileName + ".right");
		    rTrajectory = PathfinderFRC.getTrajectory("output/" + pathFileName + ".left");
        }
        dt = lTrajectory.get(0).dt;

        left = new EncoderFollower(lTrajectory);
        right = new EncoderFollower(rTrajectory);
        left.configurePIDVA(kP, kI, kD, 1 / max_velocity, kA);
		right.configurePIDVA(kP, kI, kD, 1 / max_velocity, kA);
    }

    public Path(double x, double y, double rot) {
        kP = SmartDashboard.getNumber("Path P", kP);
		kI = SmartDashboard.getNumber("Path I", kI);
		kD = SmartDashboard.getNumber("Path D", kD);
        kA = SmartDashboard.getNumber("Path A", kA);
        kG = SmartDashboard.getNumber("Path G", kG);
        flip = false;
        Waypoint[] points = new Waypoint[]{
            new Waypoint(0, 0, 0),
            new Waypoint(x, y, Pathfinder.d2r(rot))
        };
        Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.05, max_velocity, max_acceleration, max_jerk);
        Trajectory trajectory = Pathfinder.generate(points, config);
        dt = trajectory.get(0).dt;
        TankModifier modifier = new TankModifier(trajectory);
        modifier.modify(wheelbase_width);
        left = new EncoderFollower(modifier.getLeftTrajectory());
        right = new EncoderFollower(modifier.getRightTrajectory());
        left.configurePIDVA(kP, kI, kD, 1 / max_velocity, kA);
        right.configurePIDVA(kP, kI, kD, 1 / max_velocity, kA);
        for (int i = 0; i < trajectory.length(); i++) {
            Trajectory.Segment seg = trajectory.get(i);
            
            System.out.printf("%f,%f,%f,%f,%f,%f,%f,%f\n", 
                seg.dt, seg.x, seg.y, seg.position, seg.velocity, 
                seg.acceleration, seg.jerk, seg.heading);
        }
    }

    public EncoderFollower getLeftEncFollower() {
        return left;
    }
    public EncoderFollower getRightEncFollower() {
        return right;
    }
    public boolean getFlip() {
        return flip;
    }
    public String getName() {
        return pathName;
    }

    public void configSensors(int leftEncPos, int rightEncPos, double gyroHeading) {
        left.configureEncoder(leftEncPos, TICKS_PER_REV, wheel_diameter);
        right.configureEncoder(rightEncPos, TICKS_PER_REV, wheel_diameter);
        offset = gyroHeading;
    }
    public double getDT() {
        return dt;
    }

    public double[] calculateSpeeds(int leftEncPos, int rightEncPos, double heading) {
        double desired_heading = Pathfinder.r2d(left.getHeading());
        double heading_difference = Pathfinder.boundHalfDegrees(desired_heading - (heading - offset));
        double turn = kG * heading_difference;
		if (flip) {
			turn *= -1;
        }
        turn = 0;
        double left_speed = left.calculate(leftEncPos);
        double right_speed = right.calculate(rightEncPos);
        return new double[]{left_speed + turn, -right_speed + turn};
    }

    public void reset() {
        left.reset();
        right.reset();
    }
    
	//returns instructions for which path to follow
	public static String findPath(int xPos, int yPos, int x, int y) {
		String flip = "-";
		int number = 0;
		String direction = "-";
		if (x == 0 && y == 0) {
			//flip = "-";
			direction = "+";
			number = xPos * 4 + yPos;
		}
		else if (x == 3 && y == 0) {
			flip = "+";
			direction = "+";
			number = (3-xPos) * 4 + yPos;
		}
		else if (xPos == 3 && yPos == 0) {
			flip = "+";
			//direction = "-";
			number = (3-x) * 4 + y;
		}
		else if (xPos == 0 && yPos == 0) {
			//flip = "-";
			//direction = "-";
			number = x * 4 + y;
		}
		switch(number) {
			//0: Left loading station
			case 1: return flip + "rktC1" + direction;
			case 2: return flip + "rktC2" + direction;
			case 3: return flip + "rktC3" + direction;
			case 4: return flip + "csC1" + direction;
			case 5: return flip + "csC2" + direction;
			case 6: return flip + "csC3" + direction;
			case 7: return flip + "csC4" + direction;
			case 8: return flip + "csF1" + direction;
			case 9: return flip + "csF2" + direction;
			case 10: return flip + "csF3" + direction;
			case 11: return flip + "csF4" + direction;
			//12: Right loading station
			case 13: return flip + "rktF1" + direction;
			case 14: return flip + "rktF2" + direction;
			case 15: return flip + "rktF3" + direction;
			default: return "-simple";
		}
	}
}
