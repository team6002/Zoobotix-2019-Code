/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.CameraServer;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Superstructure.SuperstructureMode;
import frc.robot.subsystems.Superstructure.WantedState;
// import frc.robot.Logging;
import frc.robot.loops.*;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static ExampleSubsystem m_subsystem = new ExampleSubsystem();
  public static OI m_oi;
  public static Superstructure mSuperstructure = Superstructure.getInstance();
  public static Drive mDrive = Drive.getInstance();
  public static ControlBoard mControlBoard = ControlBoard.getInstance();
  public static IntakeArm mIntakeArm = IntakeArm.getInstance();
  public static Intake mIntake = Intake.getInstance();
  public static Claw mClaw = Claw.getInstance();
  public static ClimbingElevator mClimbingElevator = ClimbingElevator.getInstance();
  public static Arm mArm = Arm.getInstance();
  public static Elevator mElevator = Elevator.getInstance();
  
  private Looper mEnabledLooper = new Looper();

  public Robot() {
    // try
    // {
    //     Logging.CustomLogger.setup();
    // }
    // catch (Throwable e) { Logging.logException(e);}
  }

  // Timer timer = new Timer(); 
  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  // AutoModeBase mAutoMode;
  // SendableChooser<AutoModeBase> m_chooser = new SendableChooser<>();
  // private AutoModeExecutor mAutoModeExecutor;
  // private AutoModeSelector mAutoModeSelector = new AutoModeSelector();

  
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    //start Cams
    CameraServer.getInstance().startAutomaticCapture(0);
    CameraServer.getInstance().startAutomaticCapture(1);


    // m_oi = new OI();
    m_chooser.setDefaultOption("Default Auto", new ExampleCommand());
    // m_chooser.addOption("Cross Hab", new CrossHabLineMode());
    // m_chooser.addOption("Drive Forward", new CMDDriveFoward(72));//288 is 24ft
    // chooser.addOption("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", m_chooser);
    // Logging.consoleLog("Navx Angle" + mDrive.getAngle()); 

    //register loopers to big looper
    mDrive.registerEnabledLoops(mEnabledLooper);
    // mIntakeArm.registerEnabledLoops(mEnabledLooper);
    // mClimbingElevator.registerEnabledLoops(mEnabledLooper);
    // mArm.registerEnabledLoops(mEnabledLooper);
    // mClaw.registerEnabledLoops(mEnabledLooper);

    mSuperstructure.registerEnabledLoops(mEnabledLooper);

    checkPositionAll();
    // timer.reset();    
    // Logging.consoleLog();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
    SmartDashboard.putString("Match Cycle", "DISABLED");
    mEnabledLooper.stop();
    mIntakeArm.setBrakeMode(false);
    mSuperstructure.setWantedState(Superstructure.WantedState.IDLE);

    
    // timer.stop();
    // timer.reset();
    // Logging.consoleLog();
  }

  int count = 0;
  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
    if(count == 300){
      updateTelemetry();
      count = 0;
    }else{
      count++;
    }
    
    
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_chooser.getSelected();
    //AUTO CODE BELOW
    // mDrive.driveForward(48);
    
    
    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */
    
    // schedule the autonomous command (example)
    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.start();
    // }
    mIntakeArm.setBrakeMode(true);
    mEnabledLooper.start();
    checkPositionAll();
  }

  /**
   * This function is called periodically during autonomous.
   */
  // boolean haveStarted = false;

  @Override
  public void autonomousPeriodic() {
    
    Scheduler.getInstance().run();
    SmartDashboard.putString("Match Cycle", "SANDSTORM");
    teleopPeriodic();
    // mDrive.updateTelemetry();
    // Logging.consoleLog(" Target: " + mDrive.getTarget() + " Current: "  + mDrive.getVelocity() + " Error: " + mDrive.getError());
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.cancel();
    // }

    mEnabledLooper.start();
    mIntakeArm.setBrakeMode(true);
    checkPositionAll();
    // mIntakeArm.checkPosition();
    // timer.start();
  }

  /**
   * This function is called periodically during operator control.
   */
  int loop = 0;
  @Override
  public void teleopPeriodic() { 
    SmartDashboard.putString("Match Cycle", "TELEOP");
    Scheduler.getInstance().run();
    double throttle = mControlBoard.getThrottle();
    double turn = mControlBoard.getTurn();
    if(mControlBoard.getQuickturn()){
      turn = mControlBoard.getTurn()/3; //slow down turning in quick turn mode.
    }
    mDrive.setCurvatureDrive(throttle, turn, mControlBoard.getQuickturn());
    
    
    // mClimbingElevator.setOpenLoop(mControlBoard.getCElevatorPower());
    // mIntakeArm.setOpenLoop(mControlBoard.getIArmPower());

    if(mControlBoard.wantShift()){
      mDrive.shift();//toggle between high and low gear
    }

    else if(mControlBoard.getToggleForklift()){
      mClimbingElevator.toggleForklift();
    }

    else if(mControlBoard.getShipMode()){
      mSuperstructure.toggleShipMode();
    }

    //Superstructure Mode swaps
    else if(mControlBoard.toggleHatchCargoMode()){
      mSuperstructure.setWantedState(WantedState.TOGGLE_HATCH_CARGO_MODE);
    }

    //Operator controls for moving through the state machine.
    else if(mControlBoard.getForwardState()){
      mSuperstructure.setWantedState(WantedState.FORWARD_STATE);
    }
    else if (mControlBoard.getBackState()){
      mSuperstructure.setWantedState(WantedState.BACK_STATE);
    }

    //intaking button; go straight back to the intaking stage.
    else if(mControlBoard.getPrepareToIntake()){
        mSuperstructure.setWantedState(WantedState.PREPARE_INTAKE);
        mSuperstructure.getWantIntake();
    }

   
    //shooting button
    else if(mControlBoard.shoot()){
      mSuperstructure.setWantedState(WantedState.DEPLOY);
    }

    //Climbing functions (level three climb is set on the trigger button,shoot()).
    else if(mControlBoard.prepareClimb() ){
        mSuperstructure.setWantedState(WantedState.PREPARE_CLIMB);
    }
    else if(mControlBoard.wantClimbLevelThree()){
      mSuperstructure.setWantedState(WantedState.WANT_CLIMB);
    }
    else if(mControlBoard.wantClimbLevelTwo()){
        // mSuperstructure.wantSecondLevelClimb();
        mSuperstructure.setWantedState(WantedState.WANT_CLIMB_LEVEL_TWO);
    }

    else if(mControlBoard.getWantCapture()){
      mSuperstructure.setWantedState(WantedState.WANT_CAPTURE);
    }

    else if(mControlBoard.getWantLowCapture()){
      mSuperstructure.setWantedState(WantedState.WANT_LOW_CAPTURE);
    }

    else if(mControlBoard.toggleHatchPlatform()){//pov down on right drive stick
      if(!mClaw.IsPlatformExtended()){
        mClaw.extendPlatform();
      }else{
        mClaw.retractPlatform();
      }
    }

    else if(mControlBoard.retractClimbingElevator()){
      mClimbingElevator.setPosition(SuperstructureConstants.kClimbingElevatorMin);
    }
    
    else if(mControlBoard.unlockForkliftLatch()){
      mSuperstructure.unlockForklift();
    }

    else if(mControlBoard.trimUp()){
      mSuperstructure.trimUp();
    }
    else if(mControlBoard.trimDown()){
      mSuperstructure.trimDown();
    }

    else if(mControlBoard.trimElevatorUp()){
      mSuperstructure.trimElevatorUp();
    }
    else if(mControlBoard.trimElevatorDown()){
      mSuperstructure.trimElevatorDown();
    }
    
    // else if(mControlBoard.resetEnc()){
      // mClimbingElevator.resetEncoder();
      // mElevator.resetEncoder();
      // mClimbingElevator.resetWheelEncoder();
      // mClaw.resetEncoder();
      // mArm.resetEncoder();
      // mIntakeArm.resetEncoder();
    // }
    // double pwr = mControlBoard.getClimbElevator();
    // mArm.setOpenLoop(pwr);
    // mClimbingElevator.setOpenLoop(pwr);
    // double climbPwr = mControlBoard.getClimbArm();
    // mIntakeArm.setOpenLoop(climbPwr);
    if(loop == 200){
      updateTelemetry();
      loop = 0;
    }else{
      loop++;
    }
  }

  public void updateTelemetry(){
    // mDrive.updateTelemetry();
    mIntakeArm.updateTelemetry();
    // mClimbingElevator.updateTelemetry();
    mSuperstructure.updateTelemetry();
    mElevator.updateTelemetry();
    mArm.updateTelemetry();
    mClaw.updateTelemetry();
    // mIntake.updateTelemetry();
  }

  public void checkPositionAll(){//calls checkposition from each subsystem that has it.
    mClaw.checkPosition();
    mIntakeArm.checkPosition();
    mArm.checkPosition();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
