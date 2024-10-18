package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandgroups.ArmPositionCommandGroup;
import org.firstinspires.ftc.teamcode.commands.ArmAngleCommand;
import org.firstinspires.ftc.teamcode.commands.ClawCommand;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.Drivetrain;

@TeleOp(name = "Manual Drive (NEW)")
public class ManualDriveNew extends CommandOpMode {
    private final RobotHardware robot = RobotHardware.getInstance();
    private GamepadEx gamepadDriver;
    private GamepadEx gamepadOperator;
    private Drivetrain drive;

    private double loopTime = 0.0, headingTarget=0.0, turnPower=0.0;
    private boolean armAngleModeFixed=false;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        robot.init(hardwareMap);

        gamepadDriver = new GamepadEx(gamepad1);
        gamepadOperator = new GamepadEx(gamepad2);

        this.drive = new Drivetrain(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Set current arm position to start
        robot.ARM_POSITION=0;

        //Travel Position
        gamepadDriver.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new ArmPositionCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,null,1)
        );

        //Vertical Grab - Ready (only from travel position)
        gamepadDriver.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new ConditionalCommand(
                    new ArmPositionCommandGroup(robot.armAngle,0.0,robot.armWinch,0.0,robot.wrist,0.0,robot.claw,0.500,2),
                    new WaitCommand(0),
                    () -> robot.ARM_POSITION==1
                )
        );

        //Horizontal Grab - Ready (only from travel position)
        gamepadDriver.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new ConditionalCommand(
                    new ArmPositionCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw,4),
                    new WaitCommand(0),
                    () -> robot.ARM_POSITION==1
                )
        );

        //Specimen Floor Grab - Ready (only when in travel position)
        gamepadDriver.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new ConditionalCommand(
                    new ArmPositionCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw,20),
                    new WaitCommand(0),
                    () -> robot.ARM_POSITION==1
                )
        );

        //High Basket - Ready Position (only when in travel position)
        gamepadDriver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new ConditionalCommand(
                    new ArmPositionCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw,9),
                    new WaitCommand(0),
                    () -> robot.ARM_POSITION==1
                )
        );

        //Low Basket - Ready Position (only when in travel position)
        gamepadDriver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new ConditionalCommand(
                    new ArmPositionCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw,11),
                    new WaitCommand(0),
                    () -> robot.ARM_POSITION==1
                )
        );

        //Hang Specimen (High Chamber) - Ready Position (only when in travel position)
        gamepadDriver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new ConditionalCommand(
                    new ArmPositionCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw,13),
                    new WaitCommand(0),
                    () -> robot.ARM_POSITION==1
                )
        );

        //Hang Specimen (Low Chamber) - Ready Position (only when in travel position)
        gamepadDriver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new ConditionalCommand(
                    new ArmPositionCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw,16),
                    new WaitCommand(0),
                    () -> robot.ARM_POSITION==1
                )
        );

        //Climb - Ready Position (only when in travel position)
        gamepadDriver.getGamepadButton(GamepadKeys.Button.BACK).whenPressed(
                new ConditionalCommand(
                    new ArmPositionCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw,19),
                    new WaitCommand(0),
                    () -> robot.ARM_POSITION==1
                )
        );

        while (opModeInInit()) {
            telemetry.addLine("Robot Initialized");
            telemetry.update();
        }
    }

    @Override
    public void run() {
        //Make sure mode for arm angle motor is set
        if(!armAngleModeFixed) {
            robot.armAngle.setActiveRunMode();
            armAngleModeFixed=true;
        }

        CommandScheduler.getInstance().run();
        robot.periodic();

        /* ===== DRIVETRAIN ===== */

        // Read pose
        Pose2d poseEstimate = drive.getPoseEstimate();
        double headingCurrent = poseEstimate.getHeading();

        //Check for low-speed mode
        double SPEED_MULTIPLIER = 1.0;
        if (gamepad1.left_bumper) { SPEED_MULTIPLIER = 0.3;}

        Vector2d input = new Vector2d(
            -gamepad1.left_stick_y * SPEED_MULTIPLIER,
            -gamepad1.left_stick_x * SPEED_MULTIPLIER
        ).rotated(-headingCurrent);
        headingCurrent = normalizeAngle(headingCurrent);

        //Right joystick used to identify heading (only if fully articulated)
        if (Math.sqrt(gamepad1.right_stick_x*gamepad1.right_stick_x + gamepad1.right_stick_y*gamepad1.right_stick_y) > 0.90) {
            headingTarget = normalizeAngle(Math.atan2(gamepad1.right_stick_y, -gamepad1.right_stick_x) + Math.PI / 2);
        }

        //Identify error between current heading and maintain heading
        double headingError = normalizeAngle(headingTarget - headingCurrent);

        //Calculate heading correction power
        if (Math.abs(headingError) < 0.10) {
            turnPower = 0.0;
        } else if(headingError>1) {
            turnPower = 1;
        } else if(headingError<-1) {
            turnPower = -1;
        } else {
            turnPower=(headingError/Math.PI)*3;
        }

        turnPower*=Constants.Drive.TURN_POWER_MULTIPLIER;

        drive.setWeightedDrivePower(
            new Pose2d(input.getX(), input.getY(), turnPower)
        );

        drive.update();

        /* ===== CLIMB OPERATIONS ===== */

        //Raise hooks
        if(gamepadOperator.getButton(GamepadKeys.Button.DPAD_UP)) {
            if(robot.climberLeft.getCurrentPosition()<Constants.Climbers.CLIMBER_MAX) {
                robot.climberLeft.set(1.0);
            } else {
                robot.climberLeft.set(0.0);
            }

            if(robot.climberRight.getCurrentPosition()<Constants.Climbers.CLIMBER_MAX) {
                robot.climberRight.set(1.0);
            } else {
                robot.climberRight.set(0.0);
            }

        //Lower hooks
        } else if(gamepadOperator.getButton(GamepadKeys.Button.DPAD_DOWN)) {
            if (robot.climberLeft.getCurrentPosition() > Constants.Climbers.CLIMBER_MIN) {
                robot.climberLeft.set(-1.0);
            } else {
                robot.climberLeft.set(0.0);
            }

            if (robot.climberRight.getCurrentPosition() > Constants.Climbers.CLIMBER_MIN) {
                robot.climberRight.set(-1.0);
            } else {
                robot.climberRight.set(0.0);
            }

        //Nothing - set power to zero
        } else {
            robot.climberLeft.set(0.0);
            robot.climberRight.set(0.0);
        }

        //Lift
        if(gamepadOperator.getButton(GamepadKeys.Button.DPAD_RIGHT)) {
            robot.lift.setPower(0.5);
        } else if(gamepadOperator.getButton(GamepadKeys.Button.DPAD_LEFT)) {
            robot.lift.setPower(-0.5);
        } else {
            robot.lift.setPower(0.0);
        }

        /* ===== ARM/HAND/CLAW OPERATIONS ===== */

        //Right Trigger Pressed
        if(gamepadDriver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)>Constants.General.CONTROLLER_DEADBAND) {
            //Vertical Grab - Attempt
            if(robot.ARM_POSITION==2) {
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                //Lower arm to grab position
                                new ArmPositionCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,null,3),

                                //Close claw
                                new ClawCommand(robot.claw,Constants.Arm.Claw.CLAW_POSITIONS[3]),

                                //Return to ready position -- DO NOT OPEN CLAW
                                new ArmAngleCommand(robot.armAngle,Constants.Arm.Angle.ANGLE_POSITIONS[2]),
                                new InstantCommand(() -> robot.setArmPosition(2))
                        )
                );

            //Specimen Floor Grab - Attempt
            } else if(robot.ARM_POSITION==20) {
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                //Lower arm to grab position
                                new ArmPositionCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,null,21),

                                //Close claw
                                new ClawCommand(robot.claw,Constants.Arm.Claw.CLAW_POSITIONS[21]),

                                //Return to ready position -- DO NOT OPEN CLAW
                                new ArmAngleCommand(robot.armAngle,Constants.Arm.Angle.ANGLE_POSITIONS[20]),
                                new InstantCommand(() -> robot.setArmPosition(20))
                        )
                );
            }
        }

        //Left Trigger Pressed -- Failed Grab Attempt, Basket Drop, or Specimen Score (only when in ready position)
        if(gamepadDriver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)>Constants.General.CONTROLLER_DEADBAND) {
            //Vertical Grab Attempt Failed, return to ready position
            if (robot.ARM_POSITION == 2) {
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                            new ClawCommand(robot.claw,Constants.Arm.Claw.CLAW_POSITIONS[2]),
                            new InstantCommand(() -> robot.setArmPosition(2))
                        )
                );

            //High Basket - Drop
            } else if(robot.ARM_POSITION==9) {
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                //Drop Sample
                                new ClawCommand(robot.claw,Constants.Arm.Claw.CLAW_POSITIONS[10]),
                                new InstantCommand(() -> robot.setArmPosition(10)),
                                new WaitCommand(200), //Wait 0.2 seconds to make sure sample fell

                                //Go back to travel position, with arm angle moving first
                                new ArmPositionCommandGroup(robot.armAngle,0.0,robot.armWinch,0.5,robot.wrist,0.3,null,0.0,1)

                        )
                );

                //Low Basket - Drop
            } else if(robot.ARM_POSITION==11) {
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                //Drop Sample
                                new ClawCommand(robot.claw,Constants.Arm.Claw.CLAW_POSITIONS[12]),
                                new InstantCommand(() -> robot.setArmPosition(12)),
                                new WaitCommand(200), //Wait 0.2 seconds to make sure sample fell

                                //Go back to travel position, with arm angle moving first
                                new ArmPositionCommandGroup(robot.armAngle,0.0,robot.armWinch,0.5,robot.wrist,0.3,null,0.0,1)
                        )
                );

                //High Chamber - Attempt Hang
            } else if(robot.ARM_POSITION==13) {
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                //Drop Sample
                                new ArmPositionCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,null,14),
                                new WaitCommand(400),
                                new ClawCommand(robot.claw,Constants.Arm.Claw.CLAW_POSITIONS[14]),

                                //Go back to travel position, with winch and wrist moving first
                                new ArmPositionCommandGroup(robot.armAngle,0.3,robot.armWinch,0.0,robot.wrist,0.0,null,0.0,1)
                        )
                );

            }
        }

        double loop = System.nanoTime();
        telemetry.addData("Loop Speed (hz): ", 1000000000 / (loop - loopTime));
        telemetry.addData("Global Arm Position: ", robot.ARM_POSITION);
        telemetry.addData("Arm Angle Position: ", robot.armAngle.getPosition());
        telemetry.addData("Arm Angle Target: ", robot.armAngle.getTarget());
        telemetry.addData("Arm Winch Position: ", robot.armWinch.getPosition());
        telemetry.addData("Arm Winch Target: ", robot.armWinch.getTarget());
        telemetry.addData("Climber (Left) Pos: ", robot.climberLeft.getCurrentPosition());
        telemetry.addData("Climber (Right) Pos: ", robot.climberRight.getCurrentPosition());
        telemetry.addData("Lift Angle: ", robot.getLiftEncoderAngle());

        loopTime = loop;
        telemetry.update();
    }

    //Normalize angle from -pi to pi
    public double normalizeAngle(double angle) {
        if(Math.abs(angle)<=Math.PI) {
            return angle;
        }

        if(angle>(2*Math.PI)) {
            angle-=(2*Math.PI);
        } else if(angle<(-2*Math.PI)) {
            angle+=(2*Math.PI);
        }

        if(angle>Math.PI) {
            angle=-((2*Math.PI)-angle);
        } else if(angle<-Math.PI) {
            angle=angle+(2*Math.PI);
        }

        return angle;
    }
}
