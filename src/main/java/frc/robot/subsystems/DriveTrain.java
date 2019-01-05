package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class DriveTrain extends Subsystem{

    TalonSRX left_front, left_rear, right_front, right_rear;

    public boolean slavesSet = false;

    public void initDefaultCommand(){

    }

    public DriveTrain(){
        left_front = new TalonSRX(RobotMap.LEFT_FRONT);
        left_rear = new TalonSRX(RobotMap.LEFT_REAR);
        right_front = new TalonSRX(RobotMap.RIGHT_FRONT);
        right_rear = new TalonSRX(RobotMap.RIGHT_REAR);

        if(!slavesSet)
            setSlaves();
    }

    void setSlaves(){
        left_rear.follow(left_front);
        right_rear.follow(right_front);
        slavesSet = true;
    }

    public void tankDrive(double left, double right){
        left_front.set(ControlMode.PercentOutput, left);
        right_front.set(ControlMode.PercentOutput, right);
    }
}