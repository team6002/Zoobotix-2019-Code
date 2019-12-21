/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.Logging;
import frc.robot.Constants;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;
import frc.robot.loops.Looper;
import frc.lib.util.Util;

/**
 * The main arm of the robot that holds the claw
 */
public class Arm {
    private TalonSRX mArm = new TalonSRX(Constants.kArmPort);

    private int targetPos = 0;

    private static Arm mInstance = null;
    public synchronized static Arm getInstance(){
        if(mInstance == null){
            mInstance = new Arm();
        }
        return mInstance;
    }

    public enum ControlState{
        MOTION_MAGIC,
        OPEN_LOOP,
    }
    private ControlState mControlState = ControlState.OPEN_LOOP;

    public enum WantedState{
        IDLE,
        GO_TO_POSITION,
        WANT_MANUAL,
    }
    private WantedState mWantedState = WantedState.WANT_MANUAL;

    public enum SystemState{
        HOLDING_POSITION,
        MOVING_TO_POSITION,
        MANUAL,
    }
    private SystemState mSystemState = SystemState.MANUAL;

    private final Loop mLoop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            synchronized (Arm.this) {
                
            }
        }
    
        @Override
        public void onLoop(double timestamp) {
            synchronized (Arm.this) {
                SystemState newState;

                switch (mSystemState){
                    case HOLDING_POSITION:
                        newState = handleHoldingPosition();
                        break;
                    case MOVING_TO_POSITION:
                        newState = handleMovingToPosition();
                        break;
                    case MANUAL:
                        newState = handleManual();
                        break;
                    default:
                        System.out.println("Unexpected Arm State" + mSystemState);
                        // Logging.consoleLog("Unexpected Arm State" + mSystemState);
                        newState = handleHoldingPosition();
                        break; 
                }

                if(newState != mSystemState){
                    // System.out.println("Arm State " + mSystemState + " to " + newState);
                    // Logging.consoleLog("Arm State " + mSystemState + " to " + newState);
                    mSystemState = newState;
                }
                // if(mControlState == ControlState.MOTION_MAGIC){
                //     mArm.set(ControlMode.MotionMagic, targetPos);
                // }
            }
        }
    
        @Override
        public void onStop(double timestamp) {
            stop();
    
        }
    };
    public void registerEnabledLoops(ILooper in) {
        in.register(mLoop);
    }

    private SystemState handleHoldingPosition(){
        // setPosition(getPosition());
        
        switch(mWantedState){
          case IDLE:
            return SystemState.HOLDING_POSITION;
          case GO_TO_POSITION:
            return SystemState.MOVING_TO_POSITION;
          case WANT_MANUAL:
            return SystemState.MANUAL;
          default: 
            return SystemState.HOLDING_POSITION;
        }
      }
    
      private SystemState handleMovingToPosition(){
        if(isFinished()){
          setWantedState(WantedState.IDLE);
          return SystemState.HOLDING_POSITION;
        }
        // setPosition(targetPos);
        // mArm.set(ControlMode.MotionMagic, targetPos);
        switch(mWantedState){
          case IDLE:
            // setPosition(getPosition());
            return SystemState.HOLDING_POSITION;
          case GO_TO_POSITION:
            return SystemState.MOVING_TO_POSITION;
          case WANT_MANUAL:
            return SystemState.MANUAL;
          default: 
            System.out.println("Lost Arm state in: " + mSystemState);
            // Logging.consoleLog("Lost Arm state in: " + mSystemState );
            return SystemState.HOLDING_POSITION;
        }
      }
    
      private SystemState handleManual(){
        switch(mWantedState){
          case IDLE:
            return SystemState.HOLDING_POSITION;
          case GO_TO_POSITION:
            return SystemState.MOVING_TO_POSITION;
          case WANT_MANUAL:
            return SystemState.MANUAL;
          default: 
            System.out.println("Lost Arm state in: " + mSystemState);
            // Logging.consoleLog("Lost Arm state in: " + mSystemState );
            return SystemState.MANUAL;
        }
      }

    private Arm(){
        mArm.configFactoryDefault();
        mArm.setNeutralMode(NeutralMode.Brake);
        mArm.setInverted(true);
        mArm.setSensorPhase(true);
        checkPosition();

        mArm.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        // mArm.configMotionSCurveStrength(Constants.kArmSmoothing);
        
        mArm.configClosedloopRamp(Constants.kArmClosedLoopRampRate);

        mArm.config_kF(Constants.kPositionControlSlot, Constants.kArmKf);
        mArm.config_kP(Constants.kPositionControlSlot, Constants.kArmKp);
        mArm.config_kI(Constants.kPositionControlSlot, Constants.kArmKi);
        mArm.config_kD(Constants.kPositionControlSlot, Constants.kArmKd);

        mArm.configMotionCruiseVelocity(Constants.kArmCruiseVelocity);
        mArm.configMotionAcceleration(Constants.kArmAcceleration);
    }

    public void stop(){
        setOpenLoop(0);
    }

    public void setOpenLoop(double power){
        if(mControlState != ControlState.OPEN_LOOP){
            mControlState = ControlState.OPEN_LOOP;
        }

        mArm.set(ControlMode.PercentOutput, power);
        setWantedState(WantedState.WANT_MANUAL);
    }

    public void setPosition(double pos){
        if(mControlState != ControlState.MOTION_MAGIC){
            mControlState = ControlState.MOTION_MAGIC;
        }
        
        mArm.set(ControlMode.MotionMagic, pos);
        // targetPos = pos;
        setWantedState(WantedState.GO_TO_POSITION);
    }

    
    // public void setMotionMagic(int pos){
    //     if(mControlState != ControlState.MOTION_MAGIC){
    //         mControlState = ControlState.MOTION_MAGIC;
    //     }
    //     mArm.set(ControlMode.MotionMagic, pos);
    //     targetPos = pos;
    // }

    public int getPosition(){
        return mArm.getSelectedSensorPosition();
    }

    public void setWantedState(WantedState wanted){
        mWantedState = wanted;
    }

    public boolean isFinished(){
        return mControlState == mControlState.MOTION_MAGIC && 
            Util.isAlmostEqual(mArm.getSelectedSensorPosition(), targetPos, 50);//within 50 ticks of target.
    }

    private int offset = Constants.kArmOffset;
    public void checkPosition(){//reset position of relative encoder to absolute.
        // mArm.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition);
        int absPos = mArm.getSensorCollection().getPulseWidthPosition();
        // mArm.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        mArm.setSelectedSensorPosition(absPos - offset);
    }

    public void resetEncoder(){
        mArm.setSelectedSensorPosition(0);
    }

    public void updateTelemetry(){
        SmartDashboard.putNumber("M. Arm rel pos", getPosition());
        SmartDashboard.putNumber("M. Arm abs Pos", mArm.getSensorCollection().getPulseWidthPosition());
        // SmartDashboard.putNumber("M. Arm Output", mArm.getMotorOutputVoltage());
        // SmartDashboard.putNumber("M. Arm Vel", mArm.getSelectedSensorVelocity());
        // if(mControlState == ControlState.MOTION_MAGIC){
        // SmartDashboard.putNumber("M. Act Traj Pos", mArm.getActiveTrajectoryPosition());
            // SmartDashboard.putNumber("M. Act Traj vel", mArm.getActiveTrajectoryVelocity());
        // }
        
        
        // Logging.consoleLog("arm velocity " + mArm.getSelectedSensorVelocity());
    }
    
}
