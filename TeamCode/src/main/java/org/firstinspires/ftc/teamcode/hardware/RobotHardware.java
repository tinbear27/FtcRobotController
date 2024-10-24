package org.firstinspires.ftc.teamcode.hardware;

import java.util.Timer;
import java.util.TimerTask;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ArmAngleSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ArmWinchSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClimbersSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;
import org.firstinspires.ftc.teamcode.util.Encoder;

public class RobotHardware {
    //Drivetrain
    public Drivetrain drive;
    public static Pose2d currentPose = new Pose2d();

    //Arm subsystems
    public ArmAngleSubsystem armAngle;
    public ArmWinchSubsystem armWinch;
    public WristSubsystem wrist;
    public ClawSubsystem claw;

    //Climbers/Lift subystems
    public ClimbersSubsystem climbers;
    public LiftSubsystem lift;
    public static Constants.Climbers.CLIMB_STATE climbState= Constants.Climbers.CLIMB_STATE.IDLE;
    public void setClimbState(Constants.Climbers.CLIMB_STATE newClimbState) { climbState=newClimbState; }

    //Global variables
    public int ARM_POSITION=0;
    public void setArmPosition(int armPosition) {
        this.ARM_POSITION=armPosition;
    }

    private static RobotHardware instance = null;
    private boolean enabled; //Required for instance

    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }
        instance.enabled = true;
        return instance;
    }

    public void init(final HardwareMap hardwareMap, Telemetry telemetry) {
        //Drivetrain
        this.drive = new Drivetrain(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(RobotHardware.currentPose);

        //Arm subsystems
        this.armAngle = new ArmAngleSubsystem(hardwareMap,telemetry);
        this.armWinch = new ArmWinchSubsystem(hardwareMap,telemetry);
        this.wrist = new WristSubsystem(hardwareMap,telemetry);
        this.claw = new ClawSubsystem(hardwareMap,telemetry);

        //Climber/Lift subsystems
        this.climbers = new ClimbersSubsystem(hardwareMap,telemetry);
        climbers.setTarget(0);
        this.lift = new LiftSubsystem(hardwareMap,telemetry);
        lift.setTarget(0);

        //Set and hold start position
        holdStartPosition();
    }

    //Runs once per loop if called from OpMode
    //Note: Registered subsystems automatically call their own periodic method each loop
    public void periodic() {
    }

    public void holdStartPosition() {
        claw.setPositionByIndex(0);
        armWinch.setPositionByIndex(0);
        armAngle.holdStartPosition(Constants.Arm.Angle.ANGLE_POSITIONS[0]);

        Timer delayTimer = new Timer();
        delayTimer.schedule(new TimerTask() {
            @Override
            public void run() {
                wrist.setPositionByIndex(0);
            }
        }, 1000);

        this.ARM_POSITION=0;
    }
}

