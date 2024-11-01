package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commandgroups.ArmPositionCommandGroup;
import org.firstinspires.ftc.teamcode.commandgroups.ArmTravelAfterGrabCommandGroup;
import org.firstinspires.ftc.teamcode.commandgroups.ClimbHighLiftCommandGroup;
import org.firstinspires.ftc.teamcode.commandgroups.ClimbHighPoisedCommandGroup;
import org.firstinspires.ftc.teamcode.commandgroups.ClimbHighReadyCommandGroup;
import org.firstinspires.ftc.teamcode.commandgroups.ClimbLowLiftCommandGroup;
import org.firstinspires.ftc.teamcode.commandgroups.ClimbLowReadyCommandGroup;
import org.firstinspires.ftc.teamcode.commands.ArmAngleCommand;
import org.firstinspires.ftc.teamcode.commands.ClawCommand;
import org.firstinspires.ftc.teamcode.commands.ClimbersCommand;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.utility.PoseStorage;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

@TeleOp(name = "Manual Drive")
public class ManualDrive extends CommandOpMode {
    private final RobotHardware robot = RobotHardware.getInstance();
    private GamepadEx gamepadDriver;
    private GamepadEx gamepadOperator;

    private double loopTime = 0.0, headingTarget=0.0, turnPower=0.0;
    private boolean armAngleModeFixed=false;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        robot.init(hardwareMap,telemetry);

        //If pose was saved in auton, translate heading by 90 deg to match driver orientation
        if(PoseStorage.currentPose.getX()!=0.0) {
            PoseStorage.currentPose=new Pose2d(0.0,0.0,PoseStorage.currentPose.getHeading()-Math.toRadians(90.0));
        }
        robot.drive.setPoseEstimate(PoseStorage.currentPose);

        //Get controllers
        gamepadDriver = new GamepadEx(gamepad1);
        gamepadOperator = new GamepadEx(gamepad2);

        /* ===== MAP BUTTONS TO COMMANDS ===== */

        //Travel Position
        gamepadOperator.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new ConditionalCommand(
                    new ArmTravelAfterGrabCommandGroup(robot.armAngle,robot.armWinch,robot.wrist),
                    new ArmPositionCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,null,1),
                    () -> robot.ARM_POSITION==2
                )
        );

        //Vertical Grab - Ready (only from travel position)
        gamepadOperator.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new ConditionalCommand(
                    new ArmPositionCommandGroup(robot.armAngle,0.0,robot.armWinch,0.3,robot.wrist,0.0,robot.claw,0.500,2),
                    new WaitCommand(0),
                    () -> robot.ARM_POSITION==1
                )
        );

        //Specimen Floor Grab - Ready (only when in travel position)
        gamepadOperator.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new ConditionalCommand(
                    new ArmPositionCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw,20),
                    new WaitCommand(0),
                    () -> robot.ARM_POSITION==1
                )
        );

        //High Basket - Ready Position (only when in travel position)
        gamepadOperator.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new ConditionalCommand(
                    new ArmPositionCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw,9),
                    new WaitCommand(0),
                    () -> robot.ARM_POSITION==1
                )
        );

        //Low Basket - Ready Position (only when in travel position)
        gamepadOperator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new ConditionalCommand(
                    new ArmPositionCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw,11),
                    new WaitCommand(0),
                    () -> robot.ARM_POSITION==1
                )
        );

        //Hang Specimen (High Chamber) - Ready Position (only when in travel position)
        gamepadOperator.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new ConditionalCommand(
                    new ArmPositionCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw,13),
                    new WaitCommand(0),
                    () -> robot.ARM_POSITION==1
                )
        );

        //Hang Specimen (Low Chamber) - Ready Position (only when in travel position)
        gamepadOperator.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new ConditionalCommand(
                    new ArmPositionCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw,16),
                    new WaitCommand(0),
                    () -> robot.ARM_POSITION==1
                )
        );

        //Climb - Ready Position (only when in travel position)
        gamepadOperator.getGamepadButton(GamepadKeys.Button.BACK).whenPressed(
                new ConditionalCommand(
                    new ArmPositionCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw,19),
                    new WaitCommand(0),
                    () -> robot.ARM_POSITION==1
                )
        );

        //Set current arm position to start
        robot.ARM_POSITION=0;
        robot.holdStartPosition();

        while (opModeInInit()) {
            robot.armAngle.periodic();
            robot.armWinch.periodic();

            telemetry.addData("Saved Pose (X)",PoseStorage.currentPose.getX());
            telemetry.addData("Saved Pose (Y)",PoseStorage.currentPose.getY());
            telemetry.addData("Saved Pose (Heading)",Math.toDegrees(PoseStorage.currentPose.getHeading()));
            telemetry.addLine("Robot Initialized");
            telemetry.update();
        }
    }

    @Override
    public void run() {
        //First loop -- Make sure mode for arm angle motor is set and all arm subsystems set to travel position
        if(!armAngleModeFixed) {
            armAngleModeFixed=true;

            CommandScheduler.getInstance().schedule(
                new ArmPositionCommandGroup(robot.armAngle,0.0,robot.armWinch,0.0,robot.wrist,0.0,null,0.0,1)
            );
        }

        //Re-zero arm
        if(gamepad1.start) {
            robot.armZero();
        }

        //Schedule command and run robot periodic tasks
        CommandScheduler.getInstance().run();

        /* ===== DRIVETRAIN ===== */

        //Reset heading
        if(gamepadDriver.getButton(GamepadKeys.Button.BACK)) {
            robot.drive.setPoseEstimate(new Pose2d(0.0,0.0,0.0));
            headingTarget=0.0;
        }

        // Read pose
        Pose2d poseEstimate = robot.drive.getPoseEstimate();
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

        turnPower*= Constants.Drive.TURN_POWER_MULTIPLIER;
        if(robot.ARM_POSITION==2 || robot.ARM_POSITION==6 || robot.ARM_POSITION==20) {
            turnPower*=Constants.Drive.TURN_POWER_EXTENDED;
        }

        //Do not use drivetrain if actively climbing
        if(RobotHardware.climbState == Constants.Climbers.CLIMB_STATE.IDLE) {
            robot.drive.setWeightedDrivePower(
                    new Pose2d(input.getX(), input.getY(), turnPower)
            );
        } else {
            robot.drive.setWeightedDrivePower(
                    new Pose2d(0.0, 0.0, 0.0)
            );
        }

        robot.drive.update();

        /* ===== CLIMB OPERATIONS ===== */

        //Climb Sequences
        if(gamepadOperator.getButton(GamepadKeys.Button.START) && (RobotHardware.climbState != Constants.Climbers.CLIMB_STATE.IDLE || robot.ARM_POSITION==19)) {
            switch (RobotHardware.climbState) {
                case IDLE:
                    CommandScheduler.getInstance().schedule(
                            new ClimbLowReadyCommandGroup(robot.lift,robot.climbers)
                    );
                    break;
                case LOWRUNG_READY:
                    CommandScheduler.getInstance().schedule(
                            new ClimbLowLiftCommandGroup(robot.lift,robot.climbers)
                    );
                    break;
                case LOWRUNG_LIFT:
                    CommandScheduler.getInstance().schedule(
                            new ClimbHighReadyCommandGroup(robot.climbers,robot.armAngle,robot.armWinch,robot.wrist)
                    );
                    break;
                case HIGHRUNG_POISED:
                    CommandScheduler.getInstance().schedule(
                            new ClimbHighPoisedCommandGroup(robot.climbers)
                    );
                    break;
                case HIGHRUNG_READY:
                    CommandScheduler.getInstance().schedule(
                            new ClimbHighLiftCommandGroup(robot.climbers,robot.armAngle,robot.armWinch,robot.wrist)
                    );
                    break;
                case SUMMIT:
                    break;
                default:
                    break;
            };
        }

        //High rung - Retry
        if(gamepadOperator.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
            //Retry - Lower climbers below high bar
            if(RobotHardware.climbState== Constants.Climbers.CLIMB_STATE.HIGHRUNG_READY) {
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                            new ClimbersCommand(robot.climbers, Constants.Climbers.CLIMBER_HIGH_RETRY_POSITION),
                            new InstantCommand(() -> RobotHardware.getInstance().setClimbState(Constants.Climbers.CLIMB_STATE.HIGHRUNG_RETRY))
                        )
                );
            } else if(RobotHardware.climbState== Constants.Climbers.CLIMB_STATE.HIGHRUNG_RETRY) {
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new ClimbersCommand(robot.climbers, Constants.Climbers.CLIMBER_HIGH_READY_POSITION),
                                new InstantCommand(() -> RobotHardware.getInstance().setClimbState(Constants.Climbers.CLIMB_STATE.HIGHRUNG_READY))
                        )
                );
            }
        }

        /* ===== ARM/HAND/CLAW OPERATIONS ===== */

        //Right Trigger Pressed
        if(gamepadDriver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)>Constants.General.CONTROLLER_DEADBAND) {
            //Vertical Grab - Attempt
            if(robot.ARM_POSITION==2) {
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                //Lower arm to grab position
                                new ArmPositionCommandGroup(robot.armAngle,0.0,1000,robot.armWinch,0.0,0,robot.wrist,0.0,robot.claw,0.2,3),

                                //Return to ready position -- DO NOT OPEN CLAW
                                new ArmAngleCommand(robot.armAngle,Constants.Arm.Angle.ANGLE_POSITIONS[2]+Constants.Arm.Angle.ANGLE_EXTRA_TICKS_POST_GRAB),
                                new InstantCommand(() -> robot.setArmPosition(2))
                        )
                );

            //Specimen Floor Grab - Attempt
            } else if(robot.ARM_POSITION==20) {
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                //Lower arm to grab position
                                new ArmPositionCommandGroup(robot.armAngle,0.0,1000,robot.armWinch,0.0,0,robot.wrist,0.0,robot.claw,0.2,21),
                                //new ArmPositionCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,null,21),

                                //Close claw
                                //new ClawCommand(robot.claw,Constants.Arm.Claw.CLAW_POSITIONS[21]),
                                //new WaitCommand(300),

                                //Return to ready position -- DO NOT OPEN CLAW
                                new ArmAngleCommand(robot.armAngle,Constants.Arm.Angle.ANGLE_POSITIONS[20]+Constants.Arm.Angle.ANGLE_EXTRA_TICKS_POST_GRAB),
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
                   new ClawCommand(robot.claw, Constants.Arm.Claw.CLAW_OPEN)
                );

                //Floor Grab Attempt Failed, return to ready position
            } else if (robot.ARM_POSITION == 20) {
                CommandScheduler.getInstance().schedule(
                   new ClawCommand(robot.claw,Constants.Arm.Claw.CLAW_OPEN)
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

                //High Chamber - Attempt Specimen Hang
            } else if(robot.ARM_POSITION==13) {
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                //Hang Specimen
                                new ArmPositionCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,null,14).withTimeout(1200),
                                new WaitCommand(400),
                                new ClawCommand(robot.claw,Constants.Arm.Claw.CLAW_OPEN),

                                //Go back to travel position, with winch and wrist moving first
                                new ArmPositionCommandGroup(robot.armAngle,0.6,robot.armWinch,0.0,robot.wrist,0.0,null,0.0,1)
                        )
                );

                //Low Chamber - Attempt Specimen Hang
            } else if(robot.ARM_POSITION==16) {
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                //Hang Specimen
                                new ArmPositionCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,null,17).withTimeout(1200),
                                new WaitCommand(400),
                                new ClawCommand(robot.claw,Constants.Arm.Claw.CLAW_OPEN),

                                //Go back to travel position, with wrist delayed
                                new ArmPositionCommandGroup(robot.armAngle,0.0,robot.armWinch,0.0,robot.wrist,0.4,null,0.0,1)
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

        telemetry.addData("Climber State: ", RobotHardware.climbState);
        telemetry.addData("Climber Target Pos: ", robot.climbers.getTarget());
        telemetry.addData("Climber (AVG) Pos: ", robot.climbers.getAveragePosition());
        telemetry.addData("Climber (Left) Pos: ", robot.climbers.getLeftPosition());
        telemetry.addData("Climber (Right) Pos: ", robot.climbers.getRightPosition());
        telemetry.addData("Lift Pos: ", robot.lift.getCurrentPosition());

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
