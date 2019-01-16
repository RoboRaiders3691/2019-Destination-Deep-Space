package frc.robot;

public class RobotMap{

    public static final int LEFT_FRONT = 9, LEFT_REAR = 1, RIGHT_FRONT = 7, RIGHT_REAR = 0;
    public static final int PIGEON = 1;

    public static final double DEADBAND = 0.1;//Joystick deadband

    public static final double PGAIN = 0.1, IGAIN = 0, DGAIN = 0, FGAIN = 0.369475;
    public static final int SLOT_INDEX = 0, TIMEOUT = 10;
    public static final double NATIVE_UNITS = 4096;
    public static final int MAX_VELOCITY = 1466;//Per 100ms

    public static final int CENTER_FOLLOWER = 1, RIGHT_FOLLOWER = 0, LEFT_FOLLOWER = 2;

    public static final double WHEEL_DIAMETER = 6;//Inches
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER*Math.PI;
}