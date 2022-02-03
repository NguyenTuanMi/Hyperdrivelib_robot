// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public final class PID_Driveforward {
        public static final double kP                    = 0.1;
        public static final double kI                    = 0.1;
        public static final double kD                    = 0.1;
        public static final double positionTolerance     = 0.08;
        public static final double maximumIntegral       = 510;
        public static final double minimumIntegral       = 0;
        public static final double maximumInput          = 510;
        public static final double minimumInput          = 0;
        public static final double distanceperpulse      = 1; // in cm
        public static final double v_error               = 0.01;
    }
    public final class INT_DRIVEBASE {
        public static final int leftmaster               = 1;
        public static final int rightmaster              = 2;
        public static final int leftfollow               = 3;
        public static final int rightfollow              = 4;
    }
    public final class GAME_DATA {
        public static final double distance              = 510;
        public static final double auto_time             = 30; // in seconds
    }
    public final class PID_Rotation {
        public static final double kp                    = 0;
        public static final double ki                    = 0;
        public static final double kd                    = 0;
        public static final double min_angle             = 0;
        public static final double max_angle             = 360;
        public static final double tolerance             = 0.08;
        public static final double angle_minimumInput    = 0;
        public static final double angle_maximumInput    = 360;
        public static final double angle_minimumIntegral = 0;
        public static final double angle_maximumIntegral = 360;
        public static final double angle_v_error         = 0.02;
        public static final double abs_velocity          = 0.7;
    }
    public final class Collision_Detection {
        public final static double k_collision           = 0.8;
        public final static double time_out              = 0.2;
        public final static double time_elapsed          = 2;
    }
    public final class RUNNING_STATE {
        public final static int joystick_port            = 1;
        public final static int button_number            = 1;
    }
    public final class ROBOT_DATA {
        public final static double robot_weight          = 75.5;   // in kilograms
        public final static double motorunit_per_unit    = 0.437;
        public final static double ROBOT_WHEELBASE_WIDTH = 0.4;
    }
}
