/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
import frc.robot.SuperstructureConstants;
// import frc.robot.Logging;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;
import frc.robot.loops.Looper;

import frc.lib.util.Util;

/**
 * Add your docs here.
 */
public class Elevator extends Subsystem {
  public TalonSRX mElevator = new TalonSRX(Constants.kElevatorPort);
  public TalonSRX mElevatorSlave = new TalonSRX(Constants.kElevatorSlavePort);

  private int targetPos = getPosition();

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
  public enum ControlState{
    MOTION_MAGIC,
    POSITION_PID,
    OPEN_LOOP,
  }
  private ControlState mControlState = ControlState.OPEN_LOOP;

  
  private static Elevator mInstance = null; 
  public synchronized static Elevator getInstance(){
    if (mInstance == null) {
      mInstance = new Elevator();
    }
    return mInstance;
  }

  private final Loop mLoop = new Loop() {
    @Override
    public void onStart(double timestamp) {
        synchronized (Elevator.this) {
            
        }
    }

    @Override
    public void onLoop(double timestamp) {

        synchronized (Elevator.this) {
          
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
              System.out.println("Unexpected Elevator State! " + mSystemState);
              // Logging.consoleLog("Unexpected Elevator State! " + mSystemState);
              newState = handleManual();
          }

          if(newState != mSystemState){
            System.out.println("Elevator State " + mSystemState + " to " + newState);
            // Logging.consoleLog("Elevator State " + mSystemState + " to " + newState);
            mSystemState = newState;
          }

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
      // setWantedState(WantedState.IDLE);
      // return SystemState.HOLDING_POSITION;
    }

    switch(mWantedState){
      case IDLE:
        return SystemState.HOLDING_POSITION;
      case GO_TO_POSITION:
        return SystemState.MOVING_TO_POSITION;
      case WANT_MANUAL:
        return SystemState.MANUAL;
      default: 
        System.out.println("Lost Elevator state in: " + mSystemState);
        // Logging.consoleLog("Lost Elevator state in: " + mSystemState );
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
        System.out.println("Lost Elevator state in: " + mSystemState);
        // Logging.consoleLog("Lost Elevator state in: " + mSystemState );
        return SystemState.MANUAL;
    }
  }

  public Elevator(){
    mElevator.configFactoryDefault();
    mElevatorSlave.configFactoryDefault();

    mElevator.setNeutralMode(NeutralMode.Brake);
    mElevator.setInverted(true);
    mElevator.setSensorPhase(true);

    
    mElevatorSlave.follow(mElevator);
    mElevatorSlave.setNeutralMode(NeutralMode.Brake);
    mElevatorSlave.setInverted(InvertType.OpposeMaster);

    mElevator.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);


    mElevator.config_kF(Constants.kProfileID0, Constants.kElevatorKf);
    mElevator.config_kP(Constants.kProfileID0, Constants.kElevatorKp);
    mElevator.config_kI(Constants.kProfileID0, Constants.kElevatorKi);
    mElevator.config_kD(Constants.kProfileID0, Constants.kElevatorKd);

    mElevator.configMotionCruiseVelocity(Constants.kElevatorCruiseVelocity);
    mElevator.configMotionAcceleration(Constants.kElevatorAcceleration);

  }

  public void setOpenLoop(double power){
    if(mControlState != ControlState.OPEN_LOOP){
      mControlState = ControlState.OPEN_LOOP;
    }
    mElevator.set(ControlMode.PercentOutput, power);
    setWantedState(WantedState.WANT_MANUAL);
  }
  int trg = 0;
  public void setPosition(int pos){
    if(mControlState != ControlState.MOTION_MAGIC){
      mElevator.selectProfileSlot(Constants.kProfileID0, 0);
      mControlState = ControlState.MOTION_MAGIC;
    }
    if(pos > SuperstructureConstants.kElevatorMax){
      trg = SuperstructureConstants.kElevatorMax;
    }else if(pos < SuperstructureConstants.kElevatorMin){
      trg = SuperstructureConstants.kElevatorMin;
    }else{
      trg = pos;
    }
    // targetPos = pos;
    setWantedState(WantedState.GO_TO_POSITION);
    mElevator.set(ControlMode.MotionMagic, trg);
  }

  //since elevator only really needs two states, min and max, just have functions for it.
  public void setMax(){
    setPosition(SuperstructureConstants.kElevatorMax);
  }
  public void setMin(){
    setPosition(SuperstructureConstants.kElevatorMin);
  }

  public int getPosition(){
    return mElevator.getSelectedSensorPosition();
  }

  
  public void setWantedState(WantedState wanted){
    mWantedState = wanted;
  }

  public void resetEncoder(){
    mElevator.setSelectedSensorPosition(0);
  }

  public boolean isFinished(){//maybe make controlstates instead of using systemstate.
    return mSystemState == mSystemState.MOVING_TO_POSITION && 
            Util.isAlmostEqual(mElevator.getSelectedSensorPosition(), targetPos, 50);//within 50 ticks of target.
  }

  public void updateTelemetry(){
    SmartDashboard.putNumber("Elevator pos", getPosition());
    // SmartDashboard.putNumber("Slave Elevator", mElevatorSlave.getMotorOutputVoltage());
  }

  public void stop(){
    mElevator.set(ControlMode.PercentOutput, 0);
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
