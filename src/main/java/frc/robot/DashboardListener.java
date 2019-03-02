/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TableEntryListener;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.*;

public class DashboardListener {
    public static void main(String[] args) {
        new DashboardListener().run();
    }

    public void run() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTableEntry automatic = SmartDashboard.getEntry("automatic");
        NetworkTableEntry gear = SmartDashboard.getEntry("highGear");
        NetworkTableEntry forward = SmartDashboard.getEntry("forward");
        NetworkTableEntry xy = SmartDashboard.getEntry("pos_table");



        inst.startClientTeam(7034);
        
        automatic.addListener(event -> {
            Robot.m_driveTrain.auto = automatic.getBoolean(false);
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        gear.addListener(event -> {
            if (gear.getBoolean(true)) {
                Robot.m_driveTrain.gear1();
            }
            else {
                Robot.m_driveTrain.gear2();
            }
            
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        forward.addListener(event -> {
            Robot.m_driveTrain.setReversed(!forward.getBoolean(true));
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        xy.addListener(event -> {
            (new pathFollower(Robot.m_driveTrain.findPath((int) xy.getNumberArray(new Number[]{0, 0})[0], (int) xy.getNumberArray(new Number[]{0, 0})[1]))).start();
            
            System.out.println("Y Changed");
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        try {
            Thread.sleep(20);
        }
        catch (InterruptedException ex) {
            System.out.println("Listener interrupted!");
            return;
        }
    }
}
