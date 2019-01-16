/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.DriverStatus;

public class Robot extends TimedRobot {

  //Subsystems
  public static DriveTrain driveTrain = new DriveTrain(true);
  public static Input input = new Input();

  //Input
  private boolean zeroed = true;//Have the controllers been zeroed?

  //Vision
  private double vision_speed = 0.65, vision_gain = 0.25;

  @Override
  public void robotInit() {

  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {

  }

  @Override
  public void autonomousPeriodic() {

  }

  public void teleopInit() {
    zeroed = true;
    driveTrain.status = DriverStatus.Control;
  }

  int lineCount = 0;
  @Override
  public void teleopPeriodic() {
    boolean tape_detected = SmartDashboard.getBoolean("tape_detected", false);

    switch(driveTrain.status){
      case Control://We have driver control
        ControlPeriodic();

      if(tape_detected && input.getVisionButton())//Enable Vision Control
        driveTrain.status = DriverStatus.Vision;
      break;
      case Distance:
        DistancePeriodic();
      break;
      case Vision:
        VisionPeriodic();

      if(!tape_detected)//We lost the vision tape
        driveTrain.status = DriverStatus.Control;
      break;
      case Follow://We are following a line
        FollowPeriodic();
      break;
    }
  }

  public void ControlPeriodic(){
    driveTrain.tankDriveVelocity(input.getXAxis()*RobotMap.MAX_VELOCITY, -input.getYAxis()*RobotMap.MAX_VELOCITY);
    if(input.getXAxis() == 0 && input.getYAxis() == 0)
      driveTrain.setPercent(0);

    lineCount++;
    if(lineCount >= 10){
      System.out.println("Control: " + driveTrain.getLineBinary());
      lineCount = 0;
    }
    if(driveTrain.detectLine()){
      driveTrain.status = DriverStatus.Follow;
      if(input.getXAxis() != 0 || input.getYAxis() != 0)
        zeroed = false;
    }
  }

  public void DistancePeriodic(){

  }

  public void VisionPeriodic(){
    double tape_theta = SmartDashboard.getNumber("tape_theta", 0);

    

    
  }

  public void FollowPeriodic(){
    if(zeroed && (input.getXAxis() != 0 || input.getYAxis() != 0)){//We are trying to drive, overwrite line follower
      //driveTrain.status = DriverStatus.Control;
      //System.out.println("Zeroed and moving");
    }else if(!zeroed && input.getXAxis() == 0 && input.getYAxis() == 0)
      zeroed = true;

    driveTrain.followLine();
  }

  @Override
  public void testPeriodic() {
  }
}
