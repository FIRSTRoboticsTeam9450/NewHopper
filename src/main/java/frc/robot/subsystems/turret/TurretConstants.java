package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class TurretConstants{
    public static final class DeviceIDs {
        public static final int kMotorID = 28;
        public static final int kEncoderId = 29;
        public static final int kSideMotorID = -1;
    }

    public static final Transform2d OFFSET_WRT_CENTER_OF_ROBOT = new Transform2d(new Translation2d(-.1079, -.1824), new Rotation2d(0)); // .1778
    public static final double ENCODER_MAGNET_OFFSET = -0.153564453125;//0.134521484375;

    //To find the ratios, divide if reduction, multiply if increase.
    //Example, if all reductions, and 7.5 -> 7 -> 7 -> 7.5: 7.5/7/7/7.5
    public static final double ROTOR_TO_SENSOR_RATIO = 52.5;//01632653; //42.552137677193921189;// 0.02350058198; //1 cycoidal, 1 7.5:1: 0.0625850 //Both Cycoidals: 0.001827990359168
    public static final double SENSOR_TO_MECHANISM_RATIO = 0.711;

    public static final double TURRET_OFFSET_TO_ZERO = 360;
    public static final double TURRET_OFFSET_TO_TARGET = 90; // 0.25; // 1.25

    public static final double TOTAL_ROTATION_DEGREES = 320; //320
    public static final double TOTAL_ROTATION_RADIANS = Math.toRadians(TOTAL_ROTATION_DEGREES);

    public static final double DEGREE_CHANGE_PER_MOTOR_REVOLUTION = 6.857142857; //2.24;
    public static final double ABS_ENCDR_POS_TO_ANGLE = 360;

}
