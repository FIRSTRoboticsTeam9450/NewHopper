package frc.robot;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;

public final class Constants { // TODO: Replace -1 IDs with real values
    public static final class RobotConstants {
        public static final String DRIVEBASE_CANBUS_NAME = "CantDrive";
        public static final String MAIN_SYSTEMS_CANBUS_NAME = "CantShoot";
        public static final String RIO_BUS_NAME = "rio";

        public static final CANBus DRIVEBASE_CANBUS = new CANBus(DRIVEBASE_CANBUS_NAME);
        public static final CANBus MAIN_SYSTEMS_CANBUS = DRIVEBASE_CANBUS;//new CANBus(MAIN_SYSTEMS_CANBUS_NAME); // TODO: After getting a second CANivore on the robot, change RobotConstants.MAIN_SYSTEMS_CANBUS to its unique value (which is currently commented out).
        public static final CANBus RIO_BUS = new CANBus(RIO_BUS_NAME);

        public static final boolean ENABLE_FOC = true;

        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
    }

    public static final class TransferConstants {
        public static final class DeviceIDs {
            public static final int kTransferUpperMotorID = -1;
            public static final int kTransferLowerMotorID = -1;
        }
    }

    public static final class IntakeConstants {
        public static final class DeviceIDs {
            public static final int kDeployMotorID = -1;
            public static final int kIntakeLeftMotorID = -1;
            public static final int kIntakeRightMotorID = -1;
        }
    }

    public static final class ShooterConstants {
        public static final class DeviceIDs {
            public static final int kAngleMotorID = -1;
            public static final int kShooterLeftMotorID = -1;
            public static final int kShooterRightMotorID = -1;
        }
    }

    public static final class SidesweeperConstants {
        public static final class DeviceIDs {
            public static final int kRampMotorID = -1;
            public static final int kSidesweeperUpperMotorID = -1;
            public static final int kSidesweeperLowerMotorID = -1;
        }
    }

    public static final class TurretConstants {
        // TODO: Turret encoder constants (Current values are defaults)
        public static final double kTurretEncoderMagnetOffset = 0.0;
        public static final double kTurretRotorToSensorRatio = 1.0;
        public static final double kTurretSensorToMechanismRatio = 1.0;
        
        public static final class DeviceIDs {
            public static final int kTurretEncoderID = -1;
            public static final int kTurretMotorID = -1;
            public static final int kSideMotorID = -1;
        }
    }

    public static final class VisionConstants { // TODO: Vision constants (Including Limelight names)
        public static final int kDisabledThrottle = 200;
        public static final int kEnabledThrottle = 100;
        public static final int kDisabledIMUMode = IMUMode.EXTERNAL_SEED;
        public static final int kEnabledIMUMode = IMUMode.EXTERNAL_ONLY;

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

        public static final class Limelights {
            public static final class Left {
                public static final String NAME = "limelight-l";
                public static final double LOCATION_X = 0.271457252;
                public static final double LOCATION_Y = -0.26510724;
                public static final double LOCATION_Z = 0.22526;
                public static final double PITCH = 28.1;
                public static final double ROLL = 0;
                public static final double YAW = 45;
                public static final Pose3d POSE = new Pose3d(new Translation3d(LOCATION_X, LOCATION_Y, LOCATION_Z), new Rotation3d(ROLL, PITCH, YAW));
            }

            public static final class Right {
                public static final String NAME = "limelight-r";
                public static final double LOCATION_X = 0.271457252;
                public static final double LOCATION_Y = 0.26510724;
                public static final double LOCATION_Z = 0.22526;
                public static final double PITCH = 28.1;
                public static final double ROLL = 0;
                public static final double YAW = -45;
                public static final Pose3d POSE = new Pose3d(new Translation3d(LOCATION_X, LOCATION_Y, LOCATION_Z), new Rotation3d(ROLL, PITCH, YAW));
            }

            public static final class Back {
                public static final String NAME = "limelight-b";
                public static final double LOCATION_X = -0.271457252;
                public static final double LOCATION_Y = -0.26510724;
                public static final double LOCATION_Z = 0.22526;
                public static final double PITCH = 28.1;
                public static final double ROLL = 0;
                public static final double YAW = 135;
                public static final Pose3d POSE = new Pose3d(new Translation3d(LOCATION_X, LOCATION_Y, LOCATION_Z), new Rotation3d(ROLL, PITCH, YAW));
            }
        }
    }

    public static final class FieldConstants {
        public static boolean isBlueAlliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue;

        public static AprilTagFieldLayout FIELD_LAYOUT;

        static {
            FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
            FIELD_LAYOUT.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
        }

        /**
         * Ferry Poses are from blue alliance driver box perspective
         */
        public static final class ImportantFieldPoses {
            public static final Translation2d BLUE_HUB_CENTER = new Translation2d(4.6256, 4.0345);
            public static final Translation2d RED_HUB_CENTER = new Translation2d(11.9154, 4.0345);

            public static final Translation2d FERRY_TOP_TARGET = new Translation2d(2.54,7.0); //  new Translation2d(2.54, 6.1976); // Used to be .455, 6.95, 2.112, 6.432
            public static final Translation2d FERRY_BOTTOM_TARGET = new Translation2d(2.54,1.0); //(2.54, 1.8), (2.54, 6.1976)
            public static final Translation2d NEUTRAL_TARGET_BOTTOM = new Translation2d(2.396,  4.041);
            public static final Translation2d NEUTRAL_TARGET_TOP = new Translation2d(2.396,  4.041);
            public static final Translation2d FERRY_MID_RIGHT_TARGET = new Translation2d(3.418,  .5);
            public static final Translation2d FERRY_MID_LEFT_TARGET = new Translation2d(3.418,  7.6);
            public static final Translation2d FERRY_MID_TARGET = new Translation2d(2.446, 4.046);

            /* ========= HARD CODED POSES ======== */
            public static final Translation2d BLUE_LEFT_FERRY_TARGET = new Translation2d();
            public static final Translation2d BLUE_RIGHT_FERRY_TARGET = new Translation2d();

            public static final Translation2d RED_LEFT_FERRY_TARGET = new Translation2d();
            public static final Translation2d RED_RIGHT_FERRY_TARGET = new Translation2d();

            public static final Translation2d BLUE_TOWER = new Translation2d(1.063, 3.75);
            public static final Translation2d RED_TOWER = new Translation2d(15.475, 4.325);
        }

        public static final class AprilTagIDs {
            public static final int RED_BOTTOM_TRENCH_NEUTRAL_SIDE = 1;
            public static final int RED_BOTTOM_HUB_NEUTRAL_SIDE = 2;
            public static final int RED_NEUTRAL_HUB_SIDE_LOWER = 3;
            public static final int RED_NEUTRAL_HUB_SIDE_HIGHER = 4;
            public static final int RED_TOP_HUB_NEUTRAL_SIDE = 5;
            public static final int RED_TOP_TRENCH_NEUTRAL_SIDE = 6;
            public static final int RED_TOP_TRENCH_ALLIANCE_SIDE = 7;
            public static final int RED_TOP_HUB_ALLIANCE_SIDE = 8;
            public static final int RED_ALLIANCE_HUB_SIDE_HIGHER = 9;
            public static final int RED_ALLIANCE_HUB_SIDE_LOWER = 10;
            public static final int RED_BOTTOM_HUB_ALLIANCE_SIDE = 11;
            public static final int RED_BOTTOM_TRENCH_ALLIANCE_SIDE = 12;
            public static final int RED_OUTPOST_LOWER = 13;
            public static final int RED_OUTPOST_HIGHER = 14;
            public static final int RED_TOWER_LOWER = 15;
            public static final int RED_TOWER_HIGHER = 16;

            public static final int BLUE_TOP_TRENCH_NEUTRAL_SIDE = 17;
            public static final int BLUE_TOP_HUB_NEUTRAL_SIDE = 18;
            public static final int BLUE_NEUTRAL_HUB_SIDE_HIGHER = 19;
            public static final int BLUE_NEUTRAL_HUB_SIDE_LOWER = 20;
            public static final int BLUE_BOTTOM_HUB_NEUTRAL_SIDE = 21;
            public static final int BLUE_BOTTOM_TRENCH_NEUTRAL_SIDE = 22;
            public static final int BLUE_BOTTOM_TRENCH_ALLIANCE_SIDE = 23;
            public static final int BLUE_BOTTOM_HUB_ALLIANCE_SIDE = 24;
            public static final int BLUE_ALLIANCE_HUB_SIDE_LOWER = 25;
            public static final int BLUE_ALLIANCE_HUB_SIDE_HIGHER = 26;
            public static final int BLUE_TOP_HUB_ALLIANCE_SIDE = 27;
            public static final int BLUE_TOP_TRENCH_ALLIANCE_SIDE = 28;
            public static final int BLUE_OUTPOST_HIGHER = 29;
            public static final int BLUE_OUTPOST_LOWER = 30;
            public static final int BLUE_TOWER_HIGHER = 31;
            public static final int BLUE_TOWER_LOWER = 32;
        }

        public static Pose3d getTag3dPose(int tagID) {
            return FIELD_LAYOUT.getTagPose(tagID).orElseThrow(() -> {
                final String errorMsg = String.format("getTagPose called for unexpected tag, %d", tagID);
                return new RuntimeException(errorMsg);
            });
        }

        /**BLUE                      RED
         * -----------------------------
         * |    1 | 2          5 | 6   |
         * |     HUB            HUB    |
         * |    3 | 4          7 | 8   |
         * -----------------------------
         */
        public static final class TrenchManeuvering {
            public static enum TrenchLocationWRTBlueOrigin {
                CLOSE_LEFT,
                CLOSE_RIGHT,
                FAR_LEFT,
                FAR_RIGHT
            }
            
            public static final Translation2d trenchOne = new Translation2d(2.618, 7.435);
            public static final Translation2d trenchTwo = new Translation2d(6.618, 7.435);
            public static final Translation2d trenchThree = new Translation2d(2.618, 0.627);
            public static final Translation2d trenchFour = new Translation2d(6.618, 0.627);

            public static final Translation2d trenchFive = new Translation2d(9.618, 7.435);
            public static final Translation2d trenchSix = new Translation2d(14.618, 7.435);
            public static final Translation2d trenchSeven = new Translation2d(9.618, 0.627);
            public static final Translation2d trenchEight = new Translation2d(14.618, 0.627);
        }
        
        public static final class ZoneConstants {
            public static final class ZoneConstantsWelded {
                public static final Translation2d RED_CORNER = new Translation2d(16.54, 8.069326);
            }
            public static final class ZoneConstantsAndyMark {
                public static final Translation2d RED_CORNER = new Translation2d(16.513048, 8.042656);
            }
        }
    }
}
