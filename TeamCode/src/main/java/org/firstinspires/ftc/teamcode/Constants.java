package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

        //Dead wheel odometry
        public static final String ODO_LEFT_ID = "DriveLeftFront";
        public static final String ODO_RIGHT_ID = "DriveRightFront";
        public static final String ODO_CENTER_ID = "DriveLeftBack";
    }

    //Drivetrain Constants
    public static final class Drive {
        //Motor device IDs
        public static final String DRIVE_LEFT_FRONT_ID = "DriveLeftFront";
        public static final String DRIVE_LEFT_BACK_ID = "DriveLeftBack";
        public static final String DRIVE_RIGHT_FRONT_ID = "DriveRightFront";
        public static final String DRIVE_RIGHT_BACK_ID = "DriveRightBack";

        //Motor inversion
        /*
        public static final DcMotorSimple.Direction DRIVE_LEFT_FRONT_DIRECTION = DcMotorSimple.Direction.FORWARD;
        public static final DcMotorSimple.Direction DRIVE_LEFT_BACK_DIRECTION = DcMotorSimple.Direction.REVERSE;
        public static final DcMotorSimple.Direction DRIVE_RIGHT_FRONT_DIRECTION = DcMotorSimple.Direction.FORWARD;
        public static final DcMotorSimple.Direction DRIVE_RIGHT_BACK_DIRECTION = DcMotorSimple.Direction.REVERSE;
        */

        public static final Motor.GoBILDA DRIVE_MOTOR_TYPE=Motor.GoBILDA.RPM_312;
        public static final Motor.ZeroPowerBehavior DRIVE_MOTOR_ZERO=Motor.ZeroPowerBehavior.BRAKE;

        public static final boolean DRIVE_LEFT_FRONT_INVERTED = true;
        public static final boolean DRIVE_LEFT_BACK_INVERTED = true;
        public static final boolean DRIVE_RIGHT_FRONT_INVERTED = false;
        public static final boolean DRIVE_RIGHT_BACK_INVERTED = false;

        //Maximum speed multiplier (0-1.0)
        public static final double DRIVE_SPEED = 0.40;
        public static final double TURN_POWER_MULTIPLIER=1.0;

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
                "[4]  (unused)",
                "[5]  (unused)",
                "[6]  Wall Specimen Grab - Ready position",
                "[7]  Wall Specimen Grab - Attempt (start)",
                "[8]  Wall Specimen Grab - Attempt (end)",
                "[9]  High Basket - Ready position",
                "[10] High Basket - Drop",
                "[11] Low Basket - Ready position",
                "[12] Low Basket - Drop",
                "[13] Hang Specimen (High Rung) - Ready position",
                "[14] Hang Specimen (High Rung) - Attempt",
                "[15] (unused)",
                "[16] Hang Specimen (Low Rung) - Ready position",
                "[17] Hang Specimen (Low Rung) - Attempt",
                "[18] Climb - Counterweight forward",
                "[19] Climb - Ready position",
                "[20] Floor Specimen Grab - Ready Position",
                "[21] Floor Specimen Grab - Attempt Grab",
                "[22] Auton - Sample Grab - Ready Position",
                "[23] Auton - Sample Grab - Attempt",
                "[24] Auton - Park with arm touching bottom rung",
                "[25] [25] Climb - Summit position",
        };

        //Angle Constants
        public static final class Angle {
            public static final String ANGLE_ID="ArmAngle";
            public static final DcMotorSimple.Direction ANGLE_DIRECTION=DcMotorSimple.Direction.REVERSE;
            public static final double ANGLE_POWER_MAX=1.0;
            public static final double ANGLE_HOLD_POWER=0.35;
            public static final int ANGLE_HOLD_TOLERANCE=50;

            //Angle limits
            public static final int ANGLE_MIN=0; //Encoder position when arm is fully lowered
            public static final int ANGLE_MAX=2420; //Maximum allowable position
            public static final int ANGLE_HORIZONTAL=100; //Encoder position when arm is horizontal
            public static final int ANGLE_VERTICAL=2200; //Encoder position when arm is vertical

            //Feedforward constants
            public static final double ANGLE_FF_RETRACTED=0.04; //Feedforward value when arm is horizontal and arm is fully retracted
            public static final double ANGLE_FF_EXTENDED=0.12; //Feedforward value when arm is horizontal and arm is fully extended
            public static final double TICKS_PER_DEGREE= 8192/360.0;
            public static final int ANGLE_EXTRA_TICKS_POST_GRAB=100;

            //PID controller constants
            public static final double p=0.0027, i=0.0, d=0.0001;

            //Motion profile constants
            public static final double ANGLE_PROFILE_ACCEL=10000; //Ticks/sec/sec
            public static final double ANGLE_PROFILE_DECEL=5000; //Ticks/sec/sec
            public static final double ANGLE_PROFILE_VELO=10000; //Ticks/sec
            public static final int ANGLE_TOLERANCE=100;

            //Positions
            public static final int[] ANGLE_POSITIONS = new int[] {
                    1100,  // [0]  At rest (start of match)
                    950,  // [1]  Travel
                    370,  // [2]  Vertical Grab - Ready position
                    220,  // [3]  Vertical Grab - Attempt
                    0,  // [4]  (unused)
                    0,  // [5]  (unused)
                    454,  // [6]  Wall Specimen Grab - Ready position
                    454,  // [7]  Wall Specimen Grab - Attempt (start)
                    850,  // [8]  Wall Specimen Grab - Attempt (end)
                    2360,  // [9]  High Basket - Ready position
                    2360,  // [10] High Basket - Drop
                    2460,  // [11] Low Basket - Ready position
                    2460,  // [12] Low Basket - Drop
                    1750,  // [13] Hang Specimen (High Rung) - Ready position
                    1250,  // [14] Hang Specimen (High Rung) - Attempt
                    0,  // [15] (unused)
                    920,  // [16] Hang Specimen (Low Rung) - Ready position
                    350,  // [17] Hang Specimen (Low Rung) - Attempt
                    1800,  // [18] Climb - Counterweight forward
                    2370,  // [19] Climb - Ready position
                    400,  // [20] Floor Specimen Grab - Ready Position
                    260,  // [21] Floor Specimen Grab - Attempt
                    400,  // [22] Auton - Sample Grab - Ready Position
                    180,  // [23] Auton - Sample Grab - Attempt
                    1100,  // [24] Auton - Park with arm touching bottom rung
                    920,  // [25] Climb - Summit position
            };
        }

        //Winch Constants
        public static final class Winch {
            public static final String WINCH_ID="ArmWinch";
            public static final DcMotorSimple.Direction WINCH_DIRECTION=DcMotorSimple.Direction.FORWARD;

            public static final double WINCH_POWER=0.5;
            public static final int WINCH_POSITION_MIN=0;
            public static final int WINCH_POSITION_MAX=550;
            public static final int WINCH_POSITION_TOLERANCE=30;

            //Feedforward constants based on extension length and arm angle
            public static final double WINCH_FF_EXTENSION=0.06;
            public static final double WINCH_FF_ANGLE=0.03;

            //Positions
            public static final int[] WINCH_POSITIONS = new int[] {
                    0,  // [0]  At rest (start of match)
                    0,  // [1]  Travel
                    500,  // [2]  Vertical Grab - Ready position
                    500,  // [3]  Vertical Grab - Attempt
                    0,  // [4]  (unused)
                    0,  // [5]  (unused)
                    45,  // [6]  Wall Specimen Grab - Ready position
                    45,  // [7]  Wall Specimen Grab - Attempt (start)
                    48,  // [8]  Wall Specimen Grab - Attempt (end)
                    550,  // [9]  High Basket - Ready position
                    550,  // [10] High Basket - Drop
                    90,  // [11] Low Basket - Ready position
                    90,  // [12] Low Basket - Drop
                    0,  // [13] Hang Specimen (High Rung) - Ready position
                    0,  // [14] Hang Specimen (High Rung) - Attempt
                    0,  // [15] (unused)
                    0,  // [16] Hang Specimen (Low Rung) - Ready position
                    0,  // [17] Hang Specimen (Low Rung) - Attempt
                    400,  // [18] Climb - Counterweight forward
                    0,  // [19] Climb - Ready position
                    200,  // [20] Floor Specimen Grab - Ready Position
                    200,  // [21] Floor Specimen Grab - Attempt
                    250,  // [22] Auton - Sample Grab - Ready Position
                    250,  // [23] Auton - Sample Grab - Attempt
                    300,  // [24] Auton - Park with arm touching bottom rung
                    45,  // [25] Climb - Summit position
            };
        }

        //Wrist Constants
        public static final class Wrist {
            public static final String WRIST_ID="WristServo";

            //Time to finish
            public static final double WRIST_SEC_PER_ROTATION=0.7; //Seconds it takes for servo to go from position 0 to 1

            //Position at horizontal
            public static final double WRIST_HORIZ_POS=0.50;

            //Positions
            public static final double[] WRIST_POSITIONS = new double[] {
                    0.14,  // [0]  At rest (start of match)
                    0.85,  // [1]  Travel
                    0.24,  // [2]  Vertical Grab - Ready position
                    0.24,  // [3]  Vertical Grab - Attempt
                    0.0,  // [4]  (unused)
                    0.0,  // [5]  (unused)
                    0.0,  // [6]  (unused)
                    0.0,  // [7]  (unused)
                    0.0,  // [8]  (unused)
                    0.62,  // [9]  High Basket - Ready position
                    0.62,  // [10] High Basket - Drop
                    0.65,  // [11] Low Basket - Ready position
                    0.65,  // [12] Low Basket - Drop
                    0.47,  // [13] Hang Specimen (High Rung) - Ready position
                    0.47,  // [14] Hang Specimen (High Rung) - Attempt
                    0.0,  // [15] (unused)
                    0.44,  // [16] Hang Specimen (Low Rung) - Ready position
                    0.44,  // [17] Hang Specimen (Low Rung) - Attempt
                    0.30,  // [18] Climb - Counterweight forward
                    0.85,  // [19] Climb - Ready position
                    0.30,  // [20] Floor Specimen Grab - Ready Position
                    0.30,  // [21] Floor Specimen Grab - Attempt
                    0.24,  // [22] Auton - Sample Grab - Ready Position
                    0.24,  // [23] Auton - Sample Grab - Attempt
                    0.70,  // [24] Auton - Park with arm touching bottom rung
                    0.27,  // [25] Climb - Summit position
            };
        }

        //Claw Constants
        public static final class Claw {
            public static final String CLAW_ID="ClawServo";

            //Preset positions
            public static final double CLAW_OPEN=0.22;
            public static final double CLAW_CLOSED=0.68;

            //Time to finish
            public static final double CLAW_SEC_PER_ROTATION=0.5; //Seconds it takes for servo to go from position 0 to 1

            //Positions
            public static final double[] CLAW_POSITIONS = new double[] {
                    CLAW_CLOSED,  // [0]  At rest (start of match)
                    CLAW_CLOSED,  // [1]  Travel
                    CLAW_OPEN,  // [2]  Vertical Grab - Ready position
                    CLAW_CLOSED,  // [3]  Vertical Grab - Attempt
                    0.0,  // [4]  (unused)
                    0.0,  // [5]  (unused)
                    CLAW_OPEN,  // [6]  Wall Specimen Grab - Ready position
                    CLAW_CLOSED,  // [7]  Wall Specimen Grab - Attempt (start)
                    CLAW_CLOSED,  // [8]  Wall Specimen Grab - Attempt (end)
                    CLAW_CLOSED,  // [9]  High Basket - Ready position
                    CLAW_OPEN,  // [10] High Basket - Drop
                    CLAW_CLOSED,  // [11] Low Basket - Ready position
                    CLAW_OPEN,  // [12] Low Basket - Drop
                    CLAW_CLOSED,  // [13] Hang Specimen (High Rung) - Ready position
                    CLAW_OPEN,  // [14] Hang Specimen (High Rung) - Attempt
                    0.0,  // [15] (unused)
                    CLAW_CLOSED,  // [16] Hang Specimen (Low Rung) - Ready position
                    CLAW_CLOSED,  // [17] Hang Specimen (Low Rung) - Attempt
                    CLAW_CLOSED,  // [18] Climb - Counterweight forward
                    CLAW_CLOSED,  // [19] Climb - Ready position
                    CLAW_OPEN,  // [20] Floor Specimen Grab - Ready Position
                    CLAW_CLOSED,  // [21] Floor Specimen Grab - Attempt
                    CLAW_OPEN,  // [22] Auton - Sample Grab - Ready Position
                    CLAW_CLOSED,  // [23] Auton - Sample Grab - Attempt
                    CLAW_CLOSED,  // [24] Auton - Park with arm touching bottom rung
                    CLAW_CLOSED,  // [25] Climb - Summit position
            };

        }
    }

    //Climber Constants
    public static final class Climbers {
        public static enum CLIMB_STATE {
            IDLE,
            LOWRUNG_READY,
            LOWRUNG_LIFT,
            HIGHRUNG_READY,
            SUMMIT,
            HIGHRUNG_RETRY
        };

        public static final String CLIMBER_LEFT_ID="ClimberLeft";
        public static final String CLIMBER_RIGHT_ID="ClimberRight";

        //Motor directions
        public static final DcMotorSimple.Direction CLIMBER_LEFT_DIRECTION=DcMotorSimple.Direction.FORWARD;
        public static final DcMotorSimple.Direction CLIMBER_RIGHT_DIRECTION=DcMotorSimple.Direction.REVERSE;

        //Power
        public static double CLIMBER_POWER=1.0;

        //Travel limits
        public static final int CLIMBER_MIN=0;
        public static final int CLIMBER_MAX=5800;

        //Positions
        public static final int CLIMBER_START_POSITION=0;
        public static final int CLIMBER_LOW_READY_POSITION=2650; //Move hooks above lower bar
        public static final int CLIMBER_LOW_ENGAGE_POSITION=1800; //Grab bar, but do not lift yet
        public static final int CLIMBER_HIGH_READY_POSITION=5800; //Move hooks above upper bar (while suspended)
        public static final int CLIMBER_HIGH_RETRY_POSITION=3800; //Retry high hook position

        //Tolerance of error (based on average position)
        public static final int CLIMBER_POSITION_TOLERANCE=20;
    }

    //Lift Constants
    public static final class Lift {
        public static final String LIFT_ID="LiftServo";
        public static final String LIFT_ENCODER_ID="LiftEncoder";

        //Direction
        public static final DcMotorSimple.Direction LIFT_DIRECTION=DcMotorSimple.Direction.REVERSE;

        //Power
        public static double LIFT_POWER=0.7;

        //Travel limits
        public static final int LIFT_MIN=0;
        public static final int LIFT_MAX=2850;

        //Tolerance of error (degrees of turn)
        public static final int LIFT_POSITION_TOLERANCE=180;
    }

    //Autonomous Constants
    public static final class Auton {


    }
}