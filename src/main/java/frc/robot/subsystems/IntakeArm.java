/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
import frc.robot.SuperstructureConstants;
// import frc.robot.Logging;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;
import frc.robot.loops.Looper;
import frc.lib.util.Util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode; 

/**
 * Subsystem for the intake arm that is used for climbing and holding the intake out.
 */
public class IntakeArm extends Subsystem {
  private TalonSRX mIntakeArm = new TalonSRX(Constants.kIntakeArmPort);
  private VictorSPX mIntakeArmSlave = new VictorSPX(Constants.kIntakeArmSlavePort);

  private int MAX = 2770; // For Absolute Value
  private int MIN = 770; // For Absolute Value
  private int OFFSET = 770;

  public int targetPos = getPosition();

  public enum ControlState {
    POSITION_PID,
    MOTION_MAGIC,
    OPEN_LOOP,
  }
  private ControlState mControlState = ControlState.OPEN_LOOP;

  public enum WantedState {
    IDLE,
    GO_TO_POSITION,
    WANT_MANUAL,
  }
  private WantedState mWantedState = WantedState.IDLE;

  public enum SystemState {
    HOLDING_POSITION,
    MOVING_TO_POSITION,
    MANUAL,
  }
  private SystemState mSystemState = SystemState.MANUAL;

  private static IntakeArm mInstance = null; 
  public synchronized static IntakeArm getInstance(){
    if (mInstance == null) {
      mInstance = new IntakeArm();
    }
    return mInstance;
  }

  private final Loop mLoop = new Loop() {
    @Override
    public void onStart(double timestamp) {
        synchronized (IntakeArm.this) {
            // setOpenLoop(0, 0);
            setBrakeMode(true);
            // setVelocitySetpoint(0, 0);
        }
    }

    @Override
    public void onLoop(double timestamp) {
        synchronized (IntakeArm.this) {
          SystemState newState;


          switch(mSystemState) {
            case HOLDING_POSITION:
              newState = handleHoldingPosition();
              break;
            case MOVING_TO_POSITION:
              // setPositionPID(kPosition);
              newState = handleMovingToPosition();
              break;
            case MANUAL:
              newState = handleManual();
              break;
            default:
              System.out.println("Unexpected Arm State: " + mSystemState);
              // Logging.consoleLog("Unexpected Arm State: " + mSystemState);
              newState = handleManual();
          }

          if(newState != mSystemState){
            System.out.println("Intake Arm State " + mSystemState + " to " + newState);
            // Logging.consoleLog("Intake Arm State " + mSystemState + " to " + newState);
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
      setWantedState(WantedState.IDLE);
      return SystemState.HOLDING_POSITION;
    }

    switch(mWantedState){
      case IDLE:
        return SystemState.HOLDING_POSITION;
      case GO_TO_POSITION:
        return SystemState.MOVING_TO_POSITION;
      case WANT_MANUAL:
        return SystemState.MANUAL;
      default: 
        System.out.println("Lost Intake Arm state in: " + mSystemState);
        // Logging.consoleLog("Lost Intake Arm state in: " + mSystemState );
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
        System.out.println("Lost Intake Arm state in: " + mSystemState);
        // Logging.consoleLog("Lost Intake Arm state in: " + mSystemState );
        return SystemState.MANUAL;
    }
  }



  public IntakeArm(){
    mIntakeArm.setNeutralMode(NeutralMode.Brake);
    mIntakeArmSlave.setNeutralMode(NeutralMode.Brake);

    mIntakeArmSlave.follow(mIntakeArm);
    mIntakeArmSlave.setInverted(InvertType.OpposeMaster);
    
    mIntakeArm.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, Constants.kDriveTimeoutMs);
    mIntakeArm.setSensorPhase(true);
    mIntakeArm.setInverted(true);
    checkPosition();
    
    // mIntakeArm.setSelectedSensorPosition(0);
    mIntakeArm.configClosedloopRamp(0.1);

    mIntakeArm.config_kF(Constants.kIntakeArmPIDLoopID0, Constants.kIntakeArmKf, Constants.kTimeOut);
    mIntakeArm.config_kP(Constants.kIntakeArmPIDLoopID0, Constants.kIntakeArmKp, Constants.kTimeOut);
    mIntakeArm.config_kI(Constants.kIntakeArmPIDLoopID0, Constants.kIntakeArmKi, Constants.kTimeOut);
    mIntakeArm.config_kD(Constants.kIntakeArmPIDLoopID0, Constants.kIntakeArmKd, Constants.kTimeOut);

    mIntakeArm.config_kF(Constants.kIntakeArmPIDLoopID1, Constants.kIntakeArmLoadKf, Constants.kTimeOut);
    mIntakeArm.config_kP(Constants.kIntakeArmPIDLoopID1, Constants.kIntakeArmLoadKp, Constants.kTimeOut);
    mIntakeArm.config_kI(Constants.kIntakeArmPIDLoopID1, Constants.kIntakeArmLoadKi, Constants.kTimeOut);
    mIntakeArm.config_kD(Constants.kIntakeArmPIDLoopID1, Constants.kIntakeArmLoadKd, Constants.kTimeOut);

    mIntakeArm.configMotionCruiseVelocity(Constants.kIntakeArmCruiseVelocity, Constants.kTimeOut);
    mIntakeArm.configMotionAcceleration(Constants.kIntakeArmAcceleration, Constants.kTimeOut);

    mIntakeArm.selectProfileSlot(Constants.kIntakeArmPIDLoopID0, 0);
  }

  public void stopIntakeArm(){
    mIntakeArm.set(ControlMode.PercentOutput, 0);
  }

  public void setOpenLoop(double pPower){
    if(mControlState != ControlState.OPEN_LOOP){
      mControlState = ControlState.OPEN_LOOP;
    }
    setWantedState(WantedState.WANT_MANUAL);
    mIntakeArm.set(ControlMode.PercentOutput, pPower);
  }

  public synchronized void setPosition(int pos){
    if(mControlState != ControlState.MOTION_MAGIC){
      mControlState = ControlState.MOTION_MAGIC;
    }
    mIntakeArm.set(ControlMode.MotionMagic, pos);
    // targetPos = pos;
    // setWantedState(WantedState.GO_TO_POSITION);
  }

  public void selectClimbingProfile(){//modified constants for climbing.
    mIntakeArm.selectProfileSlot(Constants.kIntakeArmPIDLoopID1, 0);
    mIntakeArm.configMotionCruiseVelocity(Constants.kIntakeArmLoadCruiseVelocity, Constants.kTimeOut);
    mIntakeArm.configMotionAcceleration(Constants.kIntakeArmLoadAcceleration, Constants.kTimeOut);
  }
  public void selectIntakeProfile(){
    mIntakeArm.selectProfileSlot(Constants.kIntakeArmPIDLoopID0, 0);
  }
  //short cuts for the 2 positions that intake arm needs before endgame.
  public void stow(){
    setPosition(SuperstructureConstants.kIntakeStowed);
  }
  public void deploy(){
    setPosition(SuperstructureConstants.kIntakeDeployed);
  }
  

  private int offset = Constants.kIntakeArmOffset;
  public void checkPosition(){
    // mIntakeArm.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition, 0, Constants.kDriveTimeoutMs);
    int absPos = mIntakeArm.getSensorCollection().getPulseWidthPosition();
    // mIntakeArm.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, Constants.kDriveTimeoutMs);
    mIntakeArm.setSelectedSensorPosition(absPos - offset);
  }
  public void resetEncoder(){//only does this when the intake arm is at home position.
    // mIntakeArm.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition);
    // offset = mIntakeArm.getSensorCollection().getPulseWidthPosition();
    mIntakeArm.setSelectedSensorPosition(0);
    // mIntakeArm.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
  }

  public void setBrakeMode(boolean pBrake){
    if (pBrake == true){
      mIntakeArm.setNeutralMode(NeutralMode.Brake);
    }
    else {
      mIntakeArm.setNeutralMode(NeutralMode.Coast);
    }
  }

  public int getPosition(){
    return mIntakeArm.getSelectedSensorPosition();
  }

  public void setWantedState(WantedState wanted){
    mWantedState = wanted;
  }

  public boolean isFinished(){//maybe make controlstates instead of using systemstate.
    return mControlState == mControlState.MOTION_MAGIC && 
            Util.isAlmostEqual(mIntakeArm.getSelectedSensorPosition(), targetPos, 100);//within 50 ticks of target.
  }

  public void updateTelemetry(){
    SmartDashboard.putNumber("I.Relative Enc", getPosition());
    SmartDashboard.putNumber("I.Absolute Enc", mIntakeArm.getSensorCollection().getPulseWidthPosition());
    // SmartDashboard.putNumber("I.Motor Output", mIntakeArm.getMotorOutputVoltage());
    // SmartDashboard.putNumber("IA targetPos", mIntakeArm.getClosedLoopTarget());
    // SmartDashboard.putNumber("IA error", mIntakeArm.getClosedLoopError());
    // SmartDashboard.putString("System State", mSystemState.toString());
    // SmartDashboard.putString("Wanted State", mWantedState.toString());
    // SmartDashboard.putBoolean("IA isFinished", isFinished());
  }

  public void stop(){
    mIntakeArm.set(ControlMode.PercentOutput, 0);

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }


}
