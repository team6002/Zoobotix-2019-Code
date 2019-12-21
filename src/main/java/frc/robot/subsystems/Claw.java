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

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
// import frc.robot.Logging;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;
import frc.robot.loops.Looper;
import frc.lib.util.Util;

/**
 * Subsystem for the claw intake and claw Wrist
 */
public class Claw extends Subsystem {
  public TalonSRX mClawWrist = new TalonSRX(Constants.kClawWrist);
  public TalonSRX mClawIntake = new TalonSRX(Constants.kClawIntake);

  public Solenoid mJaw = new Solenoid(Constants.kJaw); 
  public Solenoid mBeak = new Solenoid(Constants.kHatchBeak);//change name
  public Solenoid mHatchPlatform = new Solenoid(Constants.kHatchPlatform);

  private double INTAKE_ON = -0.6;
  private double SHOOT = 0.8;
  private double HOLD = -0.25;

  private boolean IsClawOpen = false;
  private boolean IsIntakeOn = false;
  
  private int targetPos = getPosition();



  private static Claw mInstance = null; 
  public synchronized static Claw getInstance(){
    if (mInstance == null) {
      mInstance = new Claw();
    }
    return mInstance;
  }
  public enum WantedState {
    IDLE,
    GO_TO_POSITION,
    WANT_MANUAL,
  }
  private WantedState mWantedState = WantedState.WANT_MANUAL;


  public enum SystemState {
    HOLDING_POSITION,
    MOVING_TO_POSITION,
    MANUAL,
  }
  private SystemState mSystemState = SystemState.MANUAL;

  public enum ControlState {
    POSITION,
    MOTION_MAGIC,
    OPEN_LOOP,
  }

  private ControlState mControlState = ControlState.OPEN_LOOP;

  public void registerEnabledLoops(ILooper in) {
    in.register(mLoop);
  }

  private final Loop mLoop = new Loop() {
    @Override
    public void onStart(double timeStamp){
      synchronized (Claw.this){


      }
    }

    @Override
    public void onLoop(double timeStamp){
      synchronized (Claw.this) {
        SystemState newState;
        switch(mSystemState){
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
              System.out.println("Unexpected Claw State! " + mSystemState);
              // Logging.consoleLog("Unexpected Claw State! " + mSystemState);
              newState = handleManual();
        }

        if(newState != mSystemState){
          System.out.println("Claw State " + mSystemState + " to " + newState);
          // Logging.consoleLog("Claw State " + mSystemState + " to " + newState);
          mSystemState = newState;
        }
        // if(mControlState == ControlState.MOTION_MAGIC){
          // mClawWrist.set(ControlMode.MotionMagic, targetPos);
        // }
      }
    }

    @Override
    public void onStop(double timeStamp) {
      stop();
    }
  } ;

  private SystemState handleHoldingPosition(){
    // mClawWrist.set(ControlMode.Position, getPosition());
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

    switch(mWantedState){
      case IDLE:
        //hold claw wrist where it is
        // setPosition(getPosition());
        return SystemState.HOLDING_POSITION;
      case GO_TO_POSITION:
        return SystemState.MOVING_TO_POSITION;
      case WANT_MANUAL:
        return SystemState.MANUAL;
      default: 
        System.out.println("Lost claw wrist state in: " + mSystemState);
        // Logging.consoleLog("Lost claw wrist state in: " + mSystemState );
        return SystemState.HOLDING_POSITION;
    }
  }

  private SystemState handleManual(){
    switch(mWantedState){
      case IDLE:
        //stop claw wrist where it is
        // setPosition(getPosition());
        return SystemState.HOLDING_POSITION;
      case GO_TO_POSITION:
        return SystemState.MOVING_TO_POSITION;
      case WANT_MANUAL:
        return SystemState.MANUAL;
      default: 
        System.out.println("Lost claw wrist state in: " + mSystemState);
        // Logging.consoleLog("Lost claw wrist state in: " + mSystemState );
        return SystemState.MANUAL;
    }
  }


  public Claw(){
    mClawWrist.configFactoryDefault();
    mClawIntake.configFactoryDefault();
  
    mClawWrist.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    mClawWrist.setSensorPhase(false);
    mClawWrist.setInverted(false);
    checkPosition();

    mClawWrist.setNeutralMode(NeutralMode.Brake);
    mClawIntake.setNeutralMode(NeutralMode.Coast);
  
    mClawWrist.config_kF(Constants.kPositionControlSlot, Constants.kClawWristKf);
    mClawWrist.config_kP(Constants.kPositionControlSlot, Constants.kClawWristKp);
    mClawWrist.config_kI(Constants.kPositionControlSlot, Constants.kClawWristKi);
    mClawWrist.config_kD(Constants.kPositionControlSlot, Constants.kClawWristKd);

    // mClawWrist.config_kF(Constants.kMotionMagicSlot, Constants.kClawWristKf);
    // mClawWrist.config_kP(Constants.kMotionMagicSlot, Constants.kClawWristKp);
    // mClawWrist.config_kI(Constants.kMotionMagicSlot, Constants.kClawWristKi);
    // mClawWrist.config_kD(Constants.kMotionMagicSlot, Constants.kClawWristKd);

    mClawWrist.configMotionAcceleration(Constants.kClawWristAcceleration, Constants.kTimeOut);
    mClawWrist.configMotionCruiseVelocity(Constants.kClawWristCruiseVelocity, Constants.kTimeOut);

  }

  public void setWantedState(WantedState wanted){
    mWantedState = wanted;
  }
  //Wrist functions

  public void closeClaw(){
    mJaw.set(true);//true for practice
    IsClawOpen = false;
  }
  public void openClaw(){
    mJaw.set(false);//false for practice
    IsClawOpen = true;
  }

  boolean flip = false;
  public void test(){
    mJaw.set(!flip);
    flip = !flip;
    // IsClawOpen = flip;
  }

  private boolean IsBeakOpen = true;
  public void openBeak(){
    mBeak.set(false);//true shoots the hatch, false retracts it.
    IsBeakOpen = true;
  }
  public void closeBeak(){
    mBeak.set(true);
    IsBeakOpen = false;
  }

  private boolean IsPlatformExtended = false;
  public void extendPlatform(){
    mHatchPlatform.set(true);
    IsPlatformExtended = true;
  }
  public void retractPlatform(){
    mHatchPlatform.set(false);
    IsPlatformExtended = false;
  }
  public void shoot(boolean wantShoot){
    if(wantShoot){
      mClawIntake.set(ControlMode.PercentOutput, SHOOT);
      IsIntakeOn = true;
    }else{
      setOff();
    }
    
  }
  public void setWristOpenLoop(double power){
    if(mControlState != ControlState.OPEN_LOOP){
      mControlState = ControlState.OPEN_LOOP;
    }
    mClawWrist.set(ControlMode.PercentOutput, power);
  }

  public void setPosition(int pos){
    if(mControlState != ControlState.MOTION_MAGIC){
      mControlState = ControlState.MOTION_MAGIC;
      // mClawWrist.selectProfileSlot(Constants.kMotionMagicSlot, 0);
    }
    mClawWrist.set(ControlMode.MotionMagic, pos);
    // targetPos = pos;
  }

  public int getPosition(){
    return mClawWrist.getSelectedSensorPosition();
  }

  public boolean IsClawOpen(){
    return IsClawOpen;
  }
  public boolean IsBeakOpen(){
    return IsBeakOpen;
  }
  public boolean IsPlatformExtended(){
    return IsPlatformExtended;
  }

  public synchronized boolean isFinished(){
    return mControlState == mControlState.MOTION_MAGIC && 
        Util.isAlmostEqual(mClawWrist.getSelectedSensorPosition(), targetPos, 50);//within 50 ticks of target.
  }

  public void resetEncoder(){
    mClawWrist.setSelectedSensorPosition(0);
  }

  private int offset = Constants.kClawOffset;
  public void checkPosition(){//for reseting the relative encoder of the wrist pivot.
      int absPos = mClawWrist.getSensorCollection().getPulseWidthPosition();;
      mClawWrist.setSelectedSensorPosition((absPos - offset));
  }



  //Intake Functions
  private void setIntakeMotor(double pPower){
    mClawIntake.set(ControlMode.PercentOutput, pPower);
  }

  public void setOn(){
    setIntakeMotor(INTAKE_ON);
    IsIntakeOn = true;
  }
  public void shoot(){
    setIntakeMotor(SHOOT);
    IsIntakeOn = true;
  }
  public void holdCargo(){
    setIntakeMotor(HOLD);
    // IsIntakeOn = true;
  }
  public void setOff(){
    setIntakeMotor(0);
    IsIntakeOn = false;
  }
  
  public boolean getIsIntakeOn(){
    return IsIntakeOn; 
  }

  public void updateTelemetry(){
    // SmartDashboard.putBoolean("Is Claw Open", IsClawOpen);
    SmartDashboard.putNumber("C.Wrist rel pos", getPosition());
    SmartDashboard.putNumber("C.Wrist abs pos", mClawWrist.getSensorCollection().getPulseWidthPosition());
    SmartDashboard.putNumber("C. Motor Output", mClawWrist.getMotorOutputVoltage());
    
  }

  public void stop(){
    setWristOpenLoop(0);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
