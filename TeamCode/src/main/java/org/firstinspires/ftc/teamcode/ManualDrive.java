package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.commandgroups.ArmPositionCommandGroup;
import org.firstinspires.ftc.teamcode.commands.ArmAngleCommand;
import org.firstinspires.ftc.teamcode.commands.ArmWinchCommand;
import org.firstinspires.ftc.teamcode.commands.ClawCommand;
import org.firstinspires.ftc.teamcode.commands.WristCommand;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@TeleOp(name = "TeleOp - Manual Drive")
public class ManualDrive extends CommandOpMode {
    private final RobotHardware robot = RobotHardware.getInstance();
    private GamepadEx gamepadDriver;
    private GamepadEx gamepadOperator;
    private MecanumDrive drivetrain;

    private double loopTime = 0.0;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        robot.init(hardwareMap);

        gamepadDriver = new GamepadEx(gamepad1);
        gamepadOperator = new GamepadEx(gamepad2);

        drivetrain = new MecanumDrive(robot.dLeftFront, robot.dRightFront, robot.dLeftBack, robot.dRightBack);
        drivetrain.setRightSideInverted(false);

        //Map controls to commands
        /*
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
        */

        //Set current arm position to start
        robot.ARM_POSITION=0;

        //Travel Position
        gamepadDriver.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new ArmPositionCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,null,1)
        );

        //Vertical Grab - Ready (only from travel position)
        gamepadDriver.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new ConditionalCommand(
                    new ArmPositionCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw,2),
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

        //Wall Grab - Ready (only when in travel position)
        gamepadDriver.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new ConditionalCommand(
                    new ArmPositionCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw,6),
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
        CommandScheduler.getInstance().run();
        robot.periodic();

        //Check for low-speed mode
        double SPEED_MULTIPLIER=1.0;
        if(gamepadDriver.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
            SPEED_MULTIPLIER=0.3;
            telemetry.addLine("=== SLOW MODE ===");
        }

        if(gamepadDriver.getButton(GamepadKeys.Button.DPAD_LEFT)) {
            telemetry.addLine("CHANGE ARM POSITION!");
            robot.ARM_POSITION=69;
        }

        //Drivetrain
        drivetrain.driveRobotCentric(
                gamepadDriver.getLeftX()*Constants.Drive.DRIVE_SPEED*SPEED_MULTIPLIER,
                gamepadDriver.getLeftY()*Constants.Drive.DRIVE_SPEED*SPEED_MULTIPLIER,
                gamepadDriver.getRightX()*Constants.Drive.DRIVE_SPEED*SPEED_MULTIPLIER,
                false
        );

        //Climbers
        if(gamepadOperator.getButton(GamepadKeys.Button.DPAD_UP)) {
            if(robot.climberLeft.getCurrentPosition()<Constants.Climbers.CLIMBER_MAX) {
                robot.climberLeft.set(-1.0);
            } else {
                robot.climberLeft.set(0.0);
            }

            if(robot.climberRight.getCurrentPosition()<Constants.Climbers.CLIMBER_MAX) {
                robot.climberRight.set(1.0);
            } else {
                robot.climberRight.set(0.0);
            }
        } else if(gamepadOperator.getButton(GamepadKeys.Button.DPAD_DOWN)) {
            if (robot.climberLeft.getCurrentPosition() > Constants.Climbers.CLIMBER_MIN) {
                robot.climberLeft.set(1.0);
            } else {
                robot.climberLeft.set(0.0);
            }

            if (robot.climberRight.getCurrentPosition() > Constants.Climbers.CLIMBER_MIN) {
                robot.climberRight.set(-1.0);
            } else {
                robot.climberRight.set(0.0);
            }
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

        //Right Trigger Pressed
        if(gamepadDriver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)>Constants.General.CONTROLLER_DEADBAND) {
            //Vertical Grab - Attempt
            if(robot.ARM_POSITION==2) {
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                //Lower arm to grab position
                                new ArmPositionCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,null,3),
                                //new WaitCommand(300),

                                //Close claw
                                new ClawCommand(robot.claw,Constants.Arm.Claw.CLAW_POSITIONS[3]),
                                new InstantCommand(() -> robot.setArmPosition(3)),
                                //new WaitCommand(200),

                                //Return to ready position -- DO NOT OPEN CLAW
                                new ArmAngleCommand(robot.armAngle,Constants.Arm.Angle.ANGLE_POSITIONS[2]),
                                new InstantCommand(() -> robot.setArmPosition(2))
                        )
                );

            //Horizontal Grab - Attempt
            } else if(robot.ARM_POSITION==4) {
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                //Close claw
                                new ClawCommand(robot.claw,Constants.Arm.Claw.CLAW_POSITIONS[5]),
                                new InstantCommand(() -> robot.setArmPosition(5)),

                                //Return to ready position (do not open claw)
                                new ArmPositionCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,null,4),
                                new InstantCommand(() -> robot.setArmPosition(4))
                        )
                );

            //Wall Grab - Attempt
            } else if(robot.ARM_POSITION==6) {
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                //Close claw
                                new ClawCommand(robot.claw,Constants.Arm.Claw.CLAW_POSITIONS[7]),
                                new InstantCommand(() -> robot.setArmPosition(7)),

                                //Lift arm
                                new ArmPositionCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,robot.claw,8),
                                new InstantCommand(() -> robot.setArmPosition(8)),

                                //TODO: Move backwards

                                //Return to ready position -- DO NOT OPEN CLAW
                                new ArmPositionCommandGroup(robot.armAngle,robot.armWinch,robot.wrist,null,6),
                                new InstantCommand(() -> robot.setArmPosition(6))
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

            } else if(robot.ARM_POSITION==9) {
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                //Drop Sample
                                new ClawCommand(robot.claw,Constants.Arm.Claw.CLAW_POSITIONS[10]),
                                new InstantCommand(() -> robot.setArmPosition(10)),
                                new WaitCommand(200), //Wait 0.2 seconds to make sure sample fell

                                //Go back to travel position, with arm angle moving first
                                new ArmPositionCommandGroup(robot.armAngle,0.0,robot.armWinch,0.5,robot.wrist,0.0,null,0.0,1)

                                /*
                                new ParallelCommandGroup(
                                        new ArmAngleCommand(Constants.Arm.Angle.ANGLE_POSITIONS[1]),
                                        new ArmWinchCommand(Constants.Arm.Winch.WINCH_POSITIONS[1]),
                                        new WristCommand(Constants.Arm.Wrist.WRIST_POSITIONS[1]),
                                        new ClawCommand(Constants.Arm.Claw.CLAW_POSITIONS[1])
                                ),
                                new GlobalArmPositionCommand(1)
                                */
                        )
                );
            }
        }

/*
            //Horizontal Grab Attempt Failed, return to ready position
            } else if(Globals.ARM_POSITION==4) {
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new ArmAngleCommand(Constants.Arm.Angle.ANGLE_POSITIONS[4]),
                                        new ArmWinchCommand(Constants.Arm.Winch.WINCH_POSITIONS[4]),
                                        new WristCommand(Constants.Arm.Wrist.WRIST_POSITIONS[4]),
                                        new ClawCommand(Constants.Arm.Claw.CLAW_POSITIONS[4])
                                ),
                                new GlobalArmPositionCommand(4)
                        )
                );

            //Sample Drop - High Basket
            } else if(Globals.ARM_POSITION==9) {
                CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new ClawCommand(Constants.Arm.Claw.CLAW_POSITIONS[10]),
                            new GlobalArmPositionCommand(10),
                            new WaitCommand(200),
                            new ParallelCommandGroup(
                                    new ArmAngleCommand(Constants.Arm.Angle.ANGLE_POSITIONS[1]),
                                    new ArmWinchCommand(Constants.Arm.Winch.WINCH_POSITIONS[1]),
                                    new WristCommand(Constants.Arm.Wrist.WRIST_POSITIONS[1]),
                                    new ClawCommand(Constants.Arm.Claw.CLAW_POSITIONS[1])
                            ),
                            new GlobalArmPositionCommand(1)
                    )
                );

            //Sample Drop - Low Basket
            } else if(Globals.ARM_POSITION==11) {
                CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new ClawCommand(Constants.Arm.Claw.CLAW_POSITIONS[12]),
                            new GlobalArmPositionCommand(12),
                            new WaitCommand(200),
                            new ParallelCommandGroup(
                                    new ArmAngleCommand(Constants.Arm.Angle.ANGLE_POSITIONS[1]),
                                    new ArmWinchCommand(Constants.Arm.Winch.WINCH_POSITIONS[1]),
                                    new WristCommand(Constants.Arm.Wrist.WRIST_POSITIONS[1]),
                                    new ClawCommand(Constants.Arm.Claw.CLAW_POSITIONS[1])
                            ),
                            new GlobalArmPositionCommand(1)
                    )
                );

            //Specimen Hang - High Chamber
            } else if(Globals.ARM_POSITION==13) {
                CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new ParallelCommandGroup(
                                    new ArmAngleCommand(Constants.Arm.Angle.ANGLE_POSITIONS[14]),
                                    new ArmWinchCommand(Constants.Arm.Winch.WINCH_POSITIONS[14]),
                                    new WristCommand(Constants.Arm.Wrist.WRIST_POSITIONS[14]),
                                    new ClawCommand(Constants.Arm.Claw.CLAW_POSITIONS[14])
                            ),
                            new GlobalArmPositionCommand(14),
                            new WaitCommand(200),
                            new ParallelCommandGroup(
                                    new ArmAngleCommand(Constants.Arm.Angle.ANGLE_POSITIONS[15]),
                                    new ArmWinchCommand(Constants.Arm.Winch.WINCH_POSITIONS[15]),
                                    new WristCommand(Constants.Arm.Wrist.WRIST_POSITIONS[15]),
                                    new ClawCommand(Constants.Arm.Claw.CLAW_POSITIONS[15])
                            ),
                            new GlobalArmPositionCommand(15),
                            new WaitCommand(200),
                            new ParallelCommandGroup(
                                    new ArmAngleCommand(Constants.Arm.Angle.ANGLE_POSITIONS[1]),
                                    new ArmWinchCommand(Constants.Arm.Winch.WINCH_POSITIONS[1]),
                                    new WristCommand(Constants.Arm.Wrist.WRIST_POSITIONS[1]),
                                    new ClawCommand(Constants.Arm.Claw.CLAW_POSITIONS[1])
                            ),
                            new GlobalArmPositionCommand(1)
                    )
                );

            //Specimen Hang - Low Chamber
            } else if(Globals.ARM_POSITION==16) {
                CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new ParallelCommandGroup(
                                    new ArmAngleCommand(Constants.Arm.Angle.ANGLE_POSITIONS[17]),
                                    new ArmWinchCommand(Constants.Arm.Winch.WINCH_POSITIONS[17]),
                                    new WristCommand(Constants.Arm.Wrist.WRIST_POSITIONS[17]),
                                    new ClawCommand(Constants.Arm.Claw.CLAW_POSITIONS[17])
                            ),
                            new GlobalArmPositionCommand(17),
                            new WaitCommand(200),
                            new ParallelCommandGroup(
                                    new ArmAngleCommand(Constants.Arm.Angle.ANGLE_POSITIONS[18]),
                                    new ArmWinchCommand(Constants.Arm.Winch.WINCH_POSITIONS[18]),
                                    new WristCommand(Constants.Arm.Wrist.WRIST_POSITIONS[18]),
                                    new ClawCommand(Constants.Arm.Claw.CLAW_POSITIONS[18])
                            ),
                            new GlobalArmPositionCommand(18),
                            new WaitCommand(200),
                            new ParallelCommandGroup(
                                    new ArmAngleCommand(Constants.Arm.Angle.ANGLE_POSITIONS[1]),
                                    new ArmWinchCommand(Constants.Arm.Winch.WINCH_POSITIONS[1]),
                                    new WristCommand(Constants.Arm.Wrist.WRIST_POSITIONS[1]),
                                    new ClawCommand(Constants.Arm.Claw.CLAW_POSITIONS[1])
                            ),
                            new GlobalArmPositionCommand(1)
                    )
                );
            }
        }
*/

        double loop = System.nanoTime();
        telemetry.addData("Loop Speed (hz): ", 1000000000 / (loop - loopTime));
        telemetry.addData("Global Arm Position: ", robot.ARM_POSITION);
        telemetry.addData("Arm Angle Position: ", robot.armAngle.getPosition());
        telemetry.addData("Arm Angle Target: ", robot.armAngle.getTarget());
        telemetry.addData("Arm Winch Position: ", robot.armWinch.getPosition());
        telemetry.addData("Arm Winch Target: ", robot.armWinch.getTarget());
        telemetry.addData("Climber (Left) Pos: ", robot.climberLeft.getCurrentPosition());
        telemetry.addData("Climber (Right) Pos: ", robot.climberRight.getCurrentPosition());

        //Odometry
        telemetry.addData("ODO (Left): ", robot.odoLeft.getCurrentPosition());
        telemetry.addData("ODO (Right): ", robot.odoRight.getCurrentPosition());
        telemetry.addData("ODO (Center): ", robot.odoCenter.getCurrentPosition());

        loopTime = loop;
        telemetry.update();
    }
}
