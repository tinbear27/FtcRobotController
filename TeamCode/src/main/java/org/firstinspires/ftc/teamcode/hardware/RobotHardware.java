package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ArmAngleSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ArmWinchSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;
import org.firstinspires.ftc.teamcode.util.Encoder;

public class RobotHardware {
    //Drivetrain
    /*
    public DcMotorEx driveLeftFront;
    public DcMotorEx driveLeftBack;
    public DcMotorEx driveRightFront;
    public DcMotorEx driveRightBack;
    */

    //TEMP -- test for built-in mecanum drive from FTCLib
    public Motor dLeftFront;
    public Motor dLeftBack;
    public Motor dRightFront;
    public Motor dRightBack;

    //Odometry
    public Encoder odoLeft;
    public Encoder odoRight;
    public Encoder odoCenter;

    //Arm subsystems
    public ArmAngleSubsystem armAngle;
    public ArmWinchSubsystem armWinch;
    public WristSubsystem wrist;
    public ClawSubsystem claw;

    //Climbers
    public MotorEx climberLeft;
    public MotorEx climberRight;

    //Lift
    public CRServo lift;
    public AnalogInput liftEncoder;

    //Global variables
    public Constants.General.OPMODE_TYPE_LIST OPMODE_TYPE = Constants.General.OPMODE_TYPE_LIST.TELEOP;
    public Constants.General.ALLIANCE_LIST ALLIANCE = Constants.General.ALLIANCE_LIST.RED;
    public Constants.General.AUTON_CYCLE_LIST AUTON_CYCLE = Constants.General.AUTON_CYCLE_LIST.BASKET_CYCLE;
    public int ARM_POSITION=0;

    public void setArmPosition(int armPosition) {
        this.ARM_POSITION=armPosition;
    }

    private HardwareMap hardwareMap;
    private static RobotHardware instance = null;
    private boolean enabled;

    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }
        instance.enabled = true;
        return instance;
    }

    public void init(final HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        //Drivetrain
        /*
        this.driveLeftFront=hardwareMap.get(DcMotorEx.class, Constants.Drive.DRIVE_LEFT_FRONT_ID);
        driveLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveLeftFront.setDirection(Constants.Drive.DRIVE_LEFT_FRONT_DIRECTION);

        this.driveLeftBack=hardwareMap.get(DcMotorEx.class, Constants.Drive.DRIVE_LEFT_BACK_ID);
        driveLeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveLeftBack.setDirection(Constants.Drive.DRIVE_LEFT_BACK_DIRECTION);

        this.driveRightFront=hardwareMap.get(DcMotorEx.class, Constants.Drive.DRIVE_RIGHT_FRONT_ID);
        driveRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveRightFront.setDirection(Constants.Drive.DRIVE_RIGHT_FRONT_DIRECTION);

        this.driveRightBack=hardwareMap.get(DcMotorEx.class, Constants.Drive.DRIVE_RIGHT_BACK_ID);
        driveRightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveRightBack.setDirection(Constants.Drive.DRIVE_RIGHT_BACK_DIRECTION);

        this.dLeftFront=new Motor(hardwareMap, Constants.Drive.DRIVE_LEFT_FRONT_ID, Constants.Drive.DRIVE_MOTOR_TYPE);
        dLeftFront.setZeroPowerBehavior(Constants.Drive.DRIVE_MOTOR_ZERO);
        dLeftFront.setInverted(Constants.Drive.DRIVE_LEFT_FRONT_INVERTED);

        this.dLeftBack=new Motor(hardwareMap, Constants.Drive.DRIVE_LEFT_BACK_ID, Constants.Drive.DRIVE_MOTOR_TYPE);
        dLeftBack.setZeroPowerBehavior(Constants.Drive.DRIVE_MOTOR_ZERO);
        dLeftBack.setInverted(Constants.Drive.DRIVE_LEFT_BACK_INVERTED);

        this.dRightFront=new Motor(hardwareMap, Constants.Drive.DRIVE_RIGHT_FRONT_ID, Constants.Drive.DRIVE_MOTOR_TYPE);
        dRightFront.setZeroPowerBehavior(Constants.Drive.DRIVE_MOTOR_ZERO);
        dRightFront.setInverted(Constants.Drive.DRIVE_RIGHT_FRONT_INVERTED);

        this.dRightBack=new Motor(hardwareMap, Constants.Drive.DRIVE_RIGHT_BACK_ID, Constants.Drive.DRIVE_MOTOR_TYPE);
        dRightBack.setZeroPowerBehavior(Constants.Drive.DRIVE_MOTOR_ZERO);
        dRightBack.setInverted(Constants.Drive.DRIVE_RIGHT_BACK_INVERTED);

        //Odometry
        odoLeft = new Encoder(hardwareMap.get(DcMotorEx.class, Constants.Sensors.ODO_LEFT_ID));
        odoRight = new Encoder(hardwareMap.get(DcMotorEx.class, Constants.Sensors.ODO_RIGHT_ID));
        odoCenter = new Encoder(hardwareMap.get(DcMotorEx.class, Constants.Sensors.ODO_CENTER_ID));
        */

        //Arm subsystems
        this.armAngle=new ArmAngleSubsystem(hardwareMap);
        this.armWinch=new ArmWinchSubsystem(hardwareMap);
        this.wrist=new WristSubsystem(hardwareMap);
        this.claw=new ClawSubsystem(hardwareMap);

        //Set and hold start position
        holdStartPosition();

        //Climbers
        this.climberLeft=new MotorEx(hardwareMap,Constants.Climbers.CLIMBER_LEFT_ID);
        climberLeft.setZeroPowerBehavior(Constants.Climbers.CLIMBER_MOTOR_ZERO);
        climberLeft.setInverted(Constants.Climbers.CLIMBER_LEFT_INVERTED);
        climberLeft.resetEncoder();

        this.climberRight=new MotorEx(hardwareMap,Constants.Climbers.CLIMBER_RIGHT_ID);
        climberRight.setZeroPowerBehavior(Constants.Climbers.CLIMBER_MOTOR_ZERO);
        climberRight.setInverted(Constants.Climbers.CLIMBER_RIGHT_INVERTED);
        climberRight.resetEncoder();

        //Lift
        this.lift=hardwareMap.get(CRServo.class,Constants.Lift.LIFT_ID);
        lift.setDirection(Constants.Lift.LIFT_DIRECTION);
        this.liftEncoder=hardwareMap.get(AnalogInput.class, Constants.Lift.LIFT_ENCODER_ID);

    }

    public void periodic() {
        armAngle.periodic();
        armWinch.periodic();
    }

    public void holdStartPosition() {
        claw.setPositionByIndex(0);
        wrist.setPositionByIndex(0);
        armWinch.setPositionByIndex(0);
        armAngle.holdStartPosition(Constants.Arm.Angle.ANGLE_POSITIONS[0]);
        this.ARM_POSITION=0;
    }

    //Get module heading using analog encoder (TODO: Add this to new lift subsystem)
    public double getLiftEncoderAngle() {
        return (1-(this.liftEncoder.getVoltage() / 3.3)) * Math.PI*2;
    }
}
