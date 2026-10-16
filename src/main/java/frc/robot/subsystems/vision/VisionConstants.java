package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public final class VisionConstants { // TODO: Vision constants (Including Limelight names)
    public static final int kDisabledThrottle = 200;
    public static final int kEnabledThrottle = 0;
    public static final int kDisabledIMUMode = VisionConstants.IMUMode.EXTERNAL_SEED;
    public static final int kEnabledIMUMode = VisionConstants.IMUMode.EXTERNAL_ONLY;

    /** MT2 IMU mode. Info copied from Limelight docs. */
    public static final class IMUMode {
        /** No internal IMU processing. MT2 uses interpolated yaw from robot's gyro sent via SetRobotOrientation(). */
        public static final int EXTERNAL_ONLY = 0;

        /** Internal IMU offset is calibrated to match external yaw each frame (seeding). MT2 still uses external yaw for botpose. */
        public static final int EXTERNAL_SEED = 1;

        /** Uses internal IMU's fused yaw only. No external input required. */
        public static final int INTERNAL_ONLY = 2;

        /** Complementary filter fuses internal IMU with MT1 vision yaw. When MT1 gets a valid pose, it slowly corrects internal IMU drift. */
        public static final int INTERNAL_MT1_ASSIST = 3;

        /** Complementary filter fuses internal IMU with external yaw from SetRobotOrientation(). This is the recommended mode, as the internal IMU's 1khz update rate is utilized for frame-by-frame motion while the robot's IMU corrects for any drift over time. */
        public static final int INTERNAL_EXTERNAL_ASSIST = 4;
    }

    public enum Limelight {

        LEFT("limelight-l",-0.23,-0.312,0.26,40,180,90),
        RIGHT("limelight-r",-0.0862,0.3114,0.0906,35,0,-90),
        BACK("limelight-b",-0.2635,-0.109,0.3022,35,0,180),
        FRONT("limelight-f",0.1071,0.2786,0.4498,35,0,0);

        public final String NAME;
        public final double LOCATION_X;
        public final double LOCATION_Y;
        public final double LOCATION_Z;
        public final double PITCH;
        public final double ROLL;
        public final double YAW;
        public final Pose3d POSE;

        Limelight(String name, double location_x, double location_y, double location_z, double pitch, double roll, double yaw) {
            this.NAME = name;
            this.LOCATION_X = location_x;
            this.LOCATION_Y = location_y;
            this.LOCATION_Z = location_z;
            this.PITCH = pitch;
            this.ROLL = roll;
            this.YAW = yaw;
            this.POSE = new Pose3d(new Translation3d(location_x,location_y,location_z),new Rotation3d(Units.degreesToRadians(roll), Units.degreesToRadians(pitch), Units.degreesToRadians(yaw)));
        }

    }
}
