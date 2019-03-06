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
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
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
        NetworkTableEntry xTarget = SmartDashboard.getEntry("xTarget");
        NetworkTableEntry yTarget = SmartDashboard.getEntry("yTarget");
        //NetworkTableEntry xy = SmartDashboard.getEntry("position");
        NetworkTableEntry test = SmartDashboard.getEntry("test");



        inst.startClientTeam(7034);
        
        automatic.addListener(event -> {
            Robot.m_driveTrain.auto = automatic.getBoolean(false);
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        gear.addListener(event -> {
            Robot.m_driveTrain.setGear(gear.getBoolean(true));
            
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        forward.addListener(event -> {
            Robot.m_driveTrain.setReversed(!forward.getBoolean(true));
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        /*
        xy.addListener(event -> {

            Number[] pos = xy.getNumberArray(new Number[]{0, 0});
            Scheduler.getInstance().add(new PathFollower(Path.findPath(Robot.m_driveTrain.xPos, Robot.m_driveTrain.yPos, (int) pos[0], (int) pos[1])));

            System.out.println("Pos Changed");
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        test.addListener(event -> {
            Scheduler.getInstance().add(new PathToPoint(3, 3, 0));
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        */
        yTarget.addListener(event -> {
            System.out.println("y");
            Scheduler.getInstance().add(new PathFollower(Path.findPath((int) Robot.m_driveTrain.xPos, (int) Robot.m_driveTrain.yPos, (int) xTarget.getDouble(0), (int) yTarget.getDouble(0))));
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
