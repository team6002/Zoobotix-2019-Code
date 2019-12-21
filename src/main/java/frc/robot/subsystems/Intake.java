/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Constants;
// import frc.robot.Logging;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;
import frc.robot.loops.Looper;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; 

/**
 * This subsystem is the intake for cargo on the end of the intakeArm
 */
public class Intake {
    private TalonSRX mIntake = new TalonSRX(Constants.kIntakePort);

    private double INTAKE_ON = -0.8;
    private boolean isIntakeOn  = false;

    private static Intake mInstance = null; 
    public synchronized static Intake getInstance(){
        if (mInstance == null) {
          mInstance = new Intake();
        }
        return mInstance;
    }
    public Intake(){
        mIntake.configFactoryDefault();
        mIntake.setNeutralMode(NeutralMode.Coast);
        mIntake.setInverted(false);
        mIntake.setSensorPhase(false);
    }

    private void setOpenLoop(double pPower){
        mIntake.set(ControlMode.PercentOutput, pPower);
    }

    public void setOn(){
        mIntake.set(ControlMode.PercentOutput, INTAKE_ON);
        isIntakeOn = true;
    }

    public void spin(){//assist in  not getting ball stuck on it
        mIntake.set(ControlMode.PercentOutput, -0.2);
        isIntakeOn = true;
    }
    public void setOff(){
        mIntake.set(ControlMode.PercentOutput, 0);
        isIntakeOn = false;
    }

    public boolean IsIntakeOn(){
        return isIntakeOn;
    }

    public void updateTelemetry(){
        SmartDashboard.putBoolean("Intake", IsIntakeOn());
    }
    
}

