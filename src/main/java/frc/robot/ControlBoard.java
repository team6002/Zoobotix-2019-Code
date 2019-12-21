/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.LatchedBoolean;

/**
 * Add your docs here.
 */
public class ControlBoard {
  private Joystick leftDriveStick = new Joystick(2);
  private Joystick rightDriveStick = new Joystick(3);
  // private Joystick buttonBoard = new Joystick(0);// an xbox controller for the co driver.  co driver has 
                                                // the ability to switch between the various drive states.


  private LatchedBoolean intakeEdge = new LatchedBoolean();
  private LatchedBoolean intakeLoadingEdge = new LatchedBoolean();
  private LatchedBoolean stowEdge = new LatchedBoolean();
  private LatchedBoolean lowEdge = new LatchedBoolean();
  private LatchedBoolean midEdge = new LatchedBoolean();
  private LatchedBoolean highEdge = new LatchedBoolean();
  
  private LatchedBoolean captureEdge = new LatchedBoolean();

  private LatchedBoolean eleLowEdge = new LatchedBoolean();
  private LatchedBoolean eleHighEdge = new LatchedBoolean();
  // private LatchedBoolean elevatorWheelEdge = new LatchedBoolean();

  private static ControlBoard mInstance = null; 
  public synchronized static ControlBoard getInstance(){
    if (mInstance == null) {
      mInstance = new ControlBoard();
    }
    return mInstance;
  }

  private LatchedBoolean switchEdge = new LatchedBoolean();
  public boolean toggleHatchCargoMode(){//driver toggle between cargo and hatch mode
    return switchEdge.update(rightDriveStick.getRawButton(6));
  }


  private LatchedBoolean retractEdge = new LatchedBoolean();
  public boolean retractClimbingElevator(){
    return  retractEdge.update(rightDriveStick.getRawButton(10));
  }

  private LatchedBoolean platformEdge = new LatchedBoolean();
  public boolean toggleHatchPlatform(){
    return platformEdge.update(rightDriveStick.getPOV() == 180);
    // return rightDriveStick.getPOV() == 180;
  }
  
  public double getThrottle(){
      if (Math.abs(leftDriveStick.getRawAxis(1)) < 0.03) //setting dead zone
        return 0;
      else 
        return -leftDriveStick.getRawAxis(1);

  }
  
  public double getTurn(){
      if (Math.abs(rightDriveStick.getRawAxis(0)) < 0.02)
        return 0;
      else  
        return -rightDriveStick.getRawAxis(0);//negative to reverse turning for curvature drive
  }

  public double getCElevatorPower(){
    return rightDriveStick.getRawAxis(1);
  }

  public double getIArmPower(){
    return leftDriveStick.getRawAxis(1);
  }

  public boolean getQuickturn(){
    return leftDriveStick.getRawButton(4);
  }
   
  public boolean getWantCapture(){//capture ball or cargo
    return captureEdge.update(rightDriveStick.getRawButton(2));
  }

  private LatchedBoolean lowCaptureEdge = new LatchedBoolean();
  public boolean getWantLowCapture(){//capture a ball to the low position
    return lowCaptureEdge.update(rightDriveStick.getRawButton(5));
  }

  private LatchedBoolean forwardStateEdge = new LatchedBoolean();
  public boolean getForwardState(){
    return forwardStateEdge.update(rightDriveStick.getRawButton(4));
  }

  private LatchedBoolean backStateEdge = new LatchedBoolean();
  public boolean getBackState(){
    return backStateEdge.update(rightDriveStick.getRawButton(3));
  }

  private LatchedBoolean shootEdge = new LatchedBoolean();
  public boolean shoot(){
    return shootEdge.update(rightDriveStick.getRawButton(1));
    // return rightDriveStick.getRawButton(1);
  }


  private LatchedBoolean ForkliftLatchEdge = new LatchedBoolean();
  public boolean unlockForkliftLatch(){
    return ForkliftLatchEdge.update(rightDriveStick.getRawButton(7));
  }
  
  public boolean getPrepareToIntake(){
    return intakeEdge.update(leftDriveStick.getRawButton(1));
  }

  private LatchedBoolean shipModeEdge = new LatchedBoolean();
  public boolean getShipMode(){
    return shipModeEdge.update(leftDriveStick.getRawButton(3));
  }

  private LatchedBoolean forkEdge = new LatchedBoolean();
  public boolean getToggleForklift(){
    return forkEdge.update(rightDriveStick.getRawButton(8));
  }

  private LatchedBoolean prepClimbEdge = new LatchedBoolean();
  public boolean prepareClimb(){
    return prepClimbEdge.update(rightDriveStick.getRawButton(11));
  }

  private LatchedBoolean climbEdge = new LatchedBoolean();
  public boolean wantClimbLevelThree(){
    return prepClimbEdge.update(rightDriveStick.getRawButton(12));
  }

  private LatchedBoolean climbLevelTwo = new LatchedBoolean();
  public boolean wantClimbLevelTwo(){
    return climbLevelTwo.update(rightDriveStick.getRawButton(9));
  }

  private LatchedBoolean trimUpEdge = new LatchedBoolean();
  public boolean trimUp(){
    return trimUpEdge.update(leftDriveStick.getPOV() == 0);
  }
  private LatchedBoolean trimDownEdge = new LatchedBoolean();
  public boolean trimDown(){
    return trimDownEdge.update(leftDriveStick.getPOV() == 180);
  }

  private LatchedBoolean trimElevatorUpEdge = new LatchedBoolean();
  public boolean trimElevatorUp(){
    return trimElevatorUpEdge.update(leftDriveStick.getPOV() == 270);
  }
  private LatchedBoolean trimElevatorDownEdge = new LatchedBoolean();
  public boolean trimElevatorDown(){
    return trimElevatorDownEdge.update(leftDriveStick.getPOV() == 90);
  }
 
  private LatchedBoolean shiftEdge = new LatchedBoolean();
  public boolean wantShift(){
    return shiftEdge.update(leftDriveStick.getRawButton(6));
  }
  
  
}
