package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ArmAngleSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ArmWinchSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class RobotHardware {
    //Drivetrain
    public DcMotorEx driveLeftFront;
    public DcMotorEx driveLeftBack;
    public DcMotorEx driveRightFront;
    public DcMotorEx driveRightBack;

    //TEMP -- test for built-in mecanum drive from FTCLib
    public Motor dLeftFront;
    public Motor dLeftBack;
    public Motor dRightFront;
    public Motor dRightBack;

    //Arm subsystems
    public ArmAngleSubsystem armAngle;
    public ArmWinchSubsystem armWinch;
    public WristSubsystem wrist;
    public ClawSubsystem claw;

    //Climber subsystem ***TODO

    //Current arm position
    public int armPosition=0;

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
        */

        this.dLeftFront=new Motor(hardwareMap, Constants.Drive.DRIVE_LEFT_FRONT_ID, Motor.GoBILDA.RPM_312);
        dLeftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        dLeftFront.setInverted(false);

        this.dLeftBack=new Motor(hardwareMap, Constants.Drive.DRIVE_LEFT_BACK_ID, Motor.GoBILDA.RPM_312);
        dLeftBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        dLeftBack.setInverted(false);

        this.dRightFront=new Motor(hardwareMap, Constants.Drive.DRIVE_RIGHT_FRONT_ID, Motor.GoBILDA.RPM_312);
        dRightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        dRightFront.setInverted(true);

        this.dRightBack=new Motor(hardwareMap, Constants.Drive.DRIVE_RIGHT_BACK_ID, Motor.GoBILDA.RPM_312);
        dRightBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        dRightBack.setInverted(true);


        //Arm subsystems
        this.armAngle=new ArmAngleSubsystem(hardwareMap);
        this.armWinch=new ArmWinchSubsystem(hardwareMap);
        this.wrist=new WristSubsystem(hardwareMap);
        this.claw=new ClawSubsystem(hardwareMap);
    }

    public void periodic() {
        armAngle.periodic();
    }
}
