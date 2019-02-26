/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.revrobotics.CANEncoder;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

/**
 * Add your docs here.
 */
public class SparkMaxEncoder implements PIDSource{
    private PIDSourceType m_type;
    private CANEncoder source;
    
    public SparkMaxEncoder(CANEncoder enc){
        source = enc;
        m_type = PIDSourceType.kDisplacement;
    }

    public SparkMaxEncoder(CANEncoder enc, PIDSourceType type){
        source = enc;
        m_type = type;
    }

    public void setPIDSourceType(PIDSourceType type){
        m_type = type;
    }

    public PIDSourceType getPIDSourceType(){
        return m_type;
    }

    public double pidGet(){
        if(m_type == PIDSourceType.kDisplacement){
            return source.getPosition();
        }
        else{
            return source.getVelocity();
        }
    }
}
