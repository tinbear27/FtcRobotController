package org.firstinspires.ftc.teamcode.hardware;

import java.util.Timer;
import java.util.TimerTask;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ArmAngleSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ArmWinchSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClimbersSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class RobotHardware {
    //Drivetrain
    public Drivetrain drive;
    public DriveSubsystem driveSubsystem;
    public static Pose2d currentPose = new Pose2d();

    //Arm subsystems
    public ArmAngleSubsystem armAngle;
    public ArmWinchSubsystem armWinch;
    public WristSubsystem wrist;
    public ClawSubsystem claw;

    //Climbers/Lift subystems
    /*
    public ClimbersSubsystem climbers;
    public LiftSubsystem lift;
    public static Constants.Climbers.CLIMB_STATE climbState= Constants.Climbers.CLIMB_STATE.IDLE;
    public void setClimbState(Constants.Climbers.CLIMB_STATE newClimbState) { climbState=newClimbState; }
    */

    //Limit switches
    public DigitalChannel limitArmAngle;
    public DigitalChannel limitArmWinch;

    //Global variables
    public Telemetry telemetry;
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
        this.driveSubsystem=new DriveSubsystem(drive, false);

        //Arm subsystems
        this.armAngle = new ArmAngleSubsystem(hardwareMap,telemetry);
        this.armWinch = new ArmWinchSubsystem(hardwareMap,telemetry);
        this.wrist = new WristSubsystem(hardwareMap,telemetry);
        this.claw = new ClawSubsystem(hardwareMap,telemetry);

        //Climber/Lift subsystems
        /*
        this.climbers = new ClimbersSubsystem(hardwareMap,telemetry);
        climbers.setTarget(0);
        this.lift = new LiftSubsystem(hardwareMap,telemetry);
        lift.setTarget(0);
        */

        //Limit switches
        this.limitArmAngle=hardwareMap.get(DigitalChannel.class,Constants.Sensors.LIMIT_ANGLE_ID);
        this.limitArmWinch=hardwareMap.get(DigitalChannel.class,Constants.Sensors.LIMIT_WINCH_ID);

        //Telemetry
        this.telemetry=telemetry;
    }

    //Re-zero arm angle and winch
    public void armZero() {
        armWinchZero();
        armAngleZero();
    }

    //Re-zero Arm Winch
    public void armWinchZero() {
        if(limitArmWinch.getState()) {
            armWinch.reset();
        } else {
            double startTime = System.currentTimeMillis();
            boolean armWinchZeroedFlag=false;
            armWinch.setZeroPower(-0.20);

            while(!armWinchZeroedFlag) {
                if(limitArmWinch.getState() || (System.currentTimeMillis()-startTime)>1500) {
                    armWinchZeroedFlag=true;
                    armWinch.setZeroPower(0.0);
                    armWinch.reset();
                }

                telemetry.addLine("=== WINCH ZEROING ===");
                telemetry.addData("Limit Switch: ",limitArmWinch.getState());
                telemetry.addData("Time Expired: ",System.currentTimeMillis()-startTime);
                telemetry.update();
            }
        }
    }
    
    //Re-zero Arm Angle
    public void armAngleZero() {
        if(limitArmAngle.getState()) {
            armAngle.reset();
        } else {
            double startTime = System.currentTimeMillis();
            boolean armAngleZeroedFlag=false;
            armAngle.setZeroPower(-0.10);

            while(!armAngleZeroedFlag) {
                if(limitArmAngle.getState() || System.currentTimeMillis()-startTime>1500) {
                    armAngleZeroedFlag=true;
                    armAngle.setZeroPower(0.0);
                    armAngle.reset();
                }

                telemetry.addLine("=== ARM ANGLE ZEROING ===");
                telemetry.addData("Limit Switch: ",limitArmAngle.getState());
                telemetry.addData("Time Expired: ",System.currentTimeMillis()-startTime);
                telemetry.update();
            }
        }
    }

    //Runs once per loop if called from OpMode
    //Note: Registered subsystems automatically call their own periodic method each loop
    public void periodic() {
        //Nothing
    }

    public void holdStartPosition() {
        claw.setPositionByIndex(0);
        armWinch.setPositionByIndex(0);
        armAngle.setPositionByIndex(0);
        wrist.setPositionByIndex(0);

        /*
        Timer delayTimer = new Timer();
        delayTimer.schedule(new TimerTask() {
            @Override
            public void run() {
                wrist.setPositionByIndex(0);
            }
        }, 1000);
*/
        this.ARM_POSITION=0;
    }
}

