package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public final class Constants {
    //General Constants
    public static final class General {
        //Controller deadband (0=disabled)
        public static final double CONTROLLER_DEADBAND = 0.05;

        //Opmode Types
        public enum OPMODE_TYPE_LIST { AUTON, TELEOP };

        //Alliances
        public enum ALLIANCE_LIST { RED, BLUE };

        //Auton Cycle Types
        public enum AUTON_CYCLE_LIST { BASKET_CYCLE, SPECIMEN_CYCLE, PARK_OBSERVATION, NONE };
    }

    //Sensor Constants
    public static final class Sensors {
        //IMU device ID
        public static final String IMU_ID = "imu";

        //IMU orientation
        public static final RevHubOrientationOnRobot.LogoFacingDirection IMU_LOGO_ORIENTATION = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        public static final RevHubOrientationOnRobot.UsbFacingDirection IMU_USB_ORIENTATION = RevHubOrientationOnRobot.UsbFacingDirection.UP;
    }

    //Drivetrain Constants
    public static final class Drive {
        //Motor device IDs
        public static final String DRIVE_LEFT_FRONT_ID = "MotorLeftFront";
        public static final String DRIVE_LEFT_BACK_ID = "MotorLeftBack";
        public static final String DRIVE_RIGHT_FRONT_ID = "MotorRightFront";
        public static final String DRIVE_RIGHT_BACK_ID = "MotorRightBack";

        //Motor inversion
        public static final DcMotorSimple.Direction DRIVE_LEFT_FRONT_DIRECTION = DcMotorSimple.Direction.FORWARD;
        public static final DcMotorSimple.Direction DRIVE_LEFT_BACK_DIRECTION = DcMotorSimple.Direction.REVERSE;
        public static final DcMotorSimple.Direction DRIVE_RIGHT_FRONT_DIRECTION = DcMotorSimple.Direction.FORWARD;
        public static final DcMotorSimple.Direction DRIVE_RIGHT_BACK_DIRECTION = DcMotorSimple.Direction.REVERSE;

        //Motor type
        public static final Motor.GoBILDA DRIVE_MOTOR_TYPE = Motor.GoBILDA.RPM_312;

        //Maximum speed multiplier (0-1.0)
        public static final double DRIVE_SPEED = 0.7;

        //Roadrunner Drive Constants
        public static final double TICKS_PER_REV = 2000;
        public static final double MAX_RPM = 312;
        public static final boolean RUN_USING_ENCODER = false; //Uses built-in drive encoders (not-dead wheels)
        public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0, getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV));

        public static double WHEEL_RADIUS = 1.89; // in
        public static double GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
        public static double TRACK_WIDTH = 16.5; // in

        /*
         * These are the feedforward parameters used to model the drive motor behavior. If you are using
         * the built-in velocity PID, *these values are fine as is*. However, if you do not have drive
         * motor encoders or have elected not to use them for velocity control, these values should be
         * empirically tuned.
         */
        public static double kV = 1.0 / rpmToVelocity(MAX_RPM);
        public static double kA = 0;
        public static double kStatic = 0;

        /*
         * These values are used to generate the trajectories for you robot. To ensure proper operation,
         * the constraints should never exceed ~80% of the robot's actual capabilities. While Road
         * Runner is designed to enable faster autonomous motion, it is a good idea for testing to start
         * small and gradually increase them later after everything is working. All distance units are
         * inches.
         */
        public static double MAX_VEL = 30;
        public static double MAX_ACCEL = 30;
        public static double MAX_ANG_VEL = Math.toRadians(60);
        public static double MAX_ANG_ACCEL = Math.toRadians(60);

        /*
         * Adjust the orientations here to match your robot. See the FTC SDK documentation for details.
         */
        public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIR = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        public static RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIR = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        public static double encoderTicksToInches(double ticks) {
            return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
        }

        public static double rpmToVelocity(double rpm) {
            return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
        }

        public static double getMotorVelocityF(double ticksPerSecond) {
            // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
            return 32767 / ticksPerSecond;
        }

        /*  OBSOLETE -- Now grabbed from OctoQuad
        public static final String ODO_LEFT_ID = "MotorLeftFront";
        public static final String ODO_RIGHT_ID = "MotorRightFront";
        public static final String ODO_CENTER_ID = "MotorRightBack";
        */

        public static final double ODO_TRACKWIDTH = 16.25;
        public static final double ODO_TICKS_TO_INCHES = 336.88;
        public static final double ODO_CENTER_WHEEL_OFFSET = 0.2;
    }

    //Arm Constants
    public static final class Arm {

        //Position titles
        public static final String[] POSITIONS = new String[] {
                "[0] At rest (start of match)",
                "[1]  Travel",
                "[2]  Vertical Grab - Ready position",
                "[3]  Vertical Grab - Attempt",
                "[4]  Horizontal Grab - Ready position",
                "[5]  Horizontal Grab - Attempt",
                "[6]  Wall Specimen Grab - Ready position",
                "[7]  Wall Specimen Grab - Attempt (start)",
                "[8]  Wall Specimen Grab - Attempt (end)",
                "[9]  High Basket - Ready position",
                "[10] High Basket - Drop",
                "[11] Low Basket - Ready position",
                "[12] Low Basket - Drop",
                "[13] Hang Specimen (High Rung) - Ready position",
                "[14] Hang Specimen (High Rung) - Attempt",
                "[15] Hang Specimen (High Rung) - Release",
                "[16] Hang Specimen (Low Rung) - Ready position",
                "[17] Hang Specimen (Low Rung) - Attempt",
                "[18] Hang Specimen (Low Rung) - Release",
                "[19] Climb - Ready position",
                "[20] (placeholder - unused)",
                "[21] (placeholder - unused)",
                "[22] (placeholder - unused)",
                "[23] (placeholder - unused)",
                "[24] (placeholder - unused)",
                "[25] (placeholder - unused)",
        };

        //Angle Constants
        public static final class Angle {
            public static final String ANGLE_ID="ArmAngle";
            public static final DcMotorSimple.Direction ANGLE_DIRECTION=DcMotorSimple.Direction.REVERSE;
            public static final double ANGLE_POWER_MAX=1.0;

            //Angle limits
            public static final int ANGLE_MIN=0; //Encoder position when arm is fully lowered
            public static final int ANGLE_MAX=2400; //Maximum allowable position
            public static final int ANGLE_HORIZONTAL=100; //Encoder position when arm is horizontal

            //Feedforward constants
            public static final double ANGLE_FF_RETRACTED=0.045; //Feedforward value when arm is horizontal and arm is fully retracted
            public static final double ANGLE_FF_EXTENDED=0.07; //Feedforward value when arm is horizontal and arm is fully extended
            public static final double TICKS_PER_DEGREE= 8192/360.0;

            //PID controller constants
            public static final double p=0.002, i=0.0, d=0.0001;

            //Motion profile constants
            public static final double ANGLE_PROFILE_ACCEL=10000; //Ticks/sec/sec
            public static final double ANGLE_PROFILE_DECEL=8000; //Ticks/sec/sec
            public static final double ANGLE_PROFILE_VELO=10000; //Ticks/sec
            public static final int ANGLE_TOLERANCE=10;

            //Positions
            public static final int[] ANGLE_POSITIONS = new int[] {
                    0,  // [0]  At rest (start of match)
                    660,  // [1]  Travel
                    300,  // [2]  Vertical Grab - Ready position
                    205,  // [3]  Vertical Grab - Attempt
                    0,  // [4]  Horizontal Grab - Ready position
                    0,  // [5]  Horizontal Grab - Attempt
                    454,  // [6]  Wall Specimen Grab - Ready position
                    454,  // [7]  Wall Specimen Grab - Attempt (start)
                    850,  // [8]  Wall Specimen Grab - Attempt (end)
                    2260,  // [9]  High Basket - Ready position
                    2260,  // [10] High Basket - Drop
                    2360,  // [11] Low Basket - Ready position
                    2360,  // [12] Low Basket - Drop
                    1855,  // [13] Hang Specimen (High Rung) - Ready position
                    1750,  // [14] Hang Specimen (High Rung) - Attempt
                    1750,  // [15] Hang Specimen (High Rung) - Release
                    684,  // [16] Hang Specimen (Low Rung) - Ready position
                    500,  // [17] Hang Specimen (Low Rung) - Attempt
                    500,  // [18] Hang Specimen (Low Rung) - Attempt
                    2370,  // [19] Climb - Ready position
                    0,  // [20] (placeholder - unused)
                    0,  // [21] (placeholder - unused)
                    0,  // [22] (placeholder - unused)
                    0,  // [23] (placeholder - unused)
                    0,  // [24] (placeholder - unused)
                    0,  // [25] (placeholder - unused)
            };

        }

        //Winch Constants
        public static final class Winch {
            public static final String WINCH_ID="ArmWinch";
            public static final DcMotorSimple.Direction WINCH_DIRECTION=DcMotorSimple.Direction.FORWARD;

            //Positions
            public static final int[] WINCH_POSITIONS = new int[] {
                    0,  // [0]  At rest (start of match)
                    0,  // [1]  Travel
                    500,  // [2]  Vertical Grab - Ready position
                    500,  // [3]  Vertical Grab - Attempt
                    173,  // [4]  Horizontal Grab - Ready position
                    173,  // [5]  Horizontal Grab - Attempt
                    45,  // [6]  Wall Specimen Grab - Ready position
                    45,  // [7]  Wall Specimen Grab - Attempt (start)
                    48,  // [8]  Wall Specimen Grab - Attempt (end)
                    460,  // [9]  High Basket - Ready position
                    460,  // [10] High Basket - Drop
                    0,  // [11] Low Basket - Ready position
                    0,  // [12] Low Basket - Drop
                    100,  // [13] Hang Specimen (High Rung) - Ready position
                    100,  // [14] Hang Specimen (High Rung) - Attempt
                    100,  // [15] Hang Specimen (High Rung) - Release
                    40,  // [16] Hang Specimen (Low Rung) - Ready position
                    45,  // [17] Hang Specimen (Low Rung) - Attempt
                    45,  // [18] Hang Specimen (Low Rung) - Release
                    0,  // [19] Climb - Ready position
                    0,  // [20] (placeholder - unused)
                    0,  // [21] (placeholder - unused)
                    0,  // [22] (placeholder - unused)
                    0,  // [23] (placeholder - unused)
                    0,  // [24] (placeholder - unused)
                    0,  // [25] (placeholder - unused)
            };

        }

        //Wrist Constants
        public static final class Wrist {
            public static final String WRIST_ID="WristServo";

            //Positions
            public static final double[] WRIST_POSITIONS = new double[] {
                    0.05,  // [0]  At rest (start of match)
                    0.05,  // [1]  Travel
                    0.81,  // [2]  Vertical Grab - Ready position
                    0.81,  // [3]  Vertical Grab - Attempt
                    0.48,  // [4]  Horizontal Grab - Ready position
                    0.48,  // [5]  Horizontal Grab - Attempt
                    0.53,  // [6]  Wall Specimen Grab - Ready position
                    0.53,  // [7]  Wall Specimen Grab - Attempt (start)
                    0.55,  // [8]  Wall Specimen Grab - Attempt (end)
                    0.38,  // [9]  High Basket - Ready position
                    0.38,  // [10] High Basket - Drop
                    0.38,  // [11] Low Basket - Ready position
                    0.38,  // [12] Low Basket - Drop
                    0.74,  // [13] Hang Specimen (High Rung) - Ready position
                    0.74,  // [14] Hang Specimen (High Rung) - Attempt
                    0.74,  // [15] Hang Specimen (High Rung) - Release
                    0.56,  // [16] Hang Specimen (Low Rung) - Ready position
                    0.56,  // [17] Hang Specimen (Low Rung) - Attempt
                    0.56,  // [18] Hang Specimen (Low Rung) - Release
                    0.46,  // [19] Climb - Ready position
                    0.0,  // [18] (placeholder - unused)
                    0.0,  // [19] (placeholder - unused)
                    0.0,  // [20] (placeholder - unused)
                    0.0,  // [21] (placeholder - unused)
                    0.0,  // [22] (placeholder - unused)
                    0.0,  // [23] (placeholder - unused)
                    0.0,  // [24] (placeholder - unused)
                    0.0,  // [25] (placeholder - unused)
            };


        }

        //Claw Constants
        public static final class Claw {
            public static final String CLAW_ID="ClawServo";

            //Preset positions
            public static final double CLAW_OPEN=0.80;
            public static final double CLAW_CLOSED=0.24;

            //Positions
            public static final double[] CLAW_POSITIONS = new double[] {
                    CLAW_CLOSED,  // [0]  At rest (start of match)
                    CLAW_CLOSED,  // [1]  Travel
                    CLAW_OPEN,  // [2]  Vertical Grab - Ready position
                    CLAW_CLOSED,  // [3]  Vertical Grab - Attempt
                    CLAW_OPEN,  // [4]  Horizontal Grab - Ready position
                    CLAW_CLOSED,  // [5]  Horizontal Grab - Attempt
                    CLAW_OPEN,  // [6]  Wall Specimen Grab - Ready position
                    CLAW_CLOSED,  // [7]  Wall Specimen Grab - Attempt (start)
                    CLAW_CLOSED,  // [8]  Wall Specimen Grab - Attempt (end)
                    CLAW_CLOSED,  // [9]  High Basket - Ready position
                    CLAW_OPEN,  // [10] High Basket - Drop
                    CLAW_CLOSED,  // [11] Low Basket - Ready position
                    CLAW_OPEN,  // [12] Low Basket - Drop
                    CLAW_CLOSED,  // [13] Hang Specimen (High Rung) - Ready position
                    CLAW_CLOSED,  // [14] Hang Specimen (High Rung) - Attempt
                    CLAW_OPEN,  // [15] Hang Specimen (High Rung) - Release
                    CLAW_CLOSED,  // [16] Hang Specimen (Low Rung) - Ready position
                    CLAW_CLOSED,  // [17] Hang Specimen (Low Rung) - Attempt
                    CLAW_OPEN,  // [18] Hang Specimen (Low Rung) - Release
                    CLAW_CLOSED,  // [19] Climb - Ready position
                    0.0,  // [20] (placeholder - unused)
                    0.0,  // [21] (placeholder - unused)
                    0.0,  // [22] (placeholder - unused)
                    0.0,  // [23] (placeholder - unused)
                    0.0,  // [24] (placeholder - unused)
                    0.0,  // [25] (placeholder - unused)
            };

        }
    }

    //Climber Constants
    public static final class Climber {

    }

    //Autonomous Constants
    public static final class Auton {


    }
}