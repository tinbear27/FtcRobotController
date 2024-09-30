package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.commands.ArmAngleCommand;
import org.firstinspires.ftc.teamcode.commands.ArmWinchCommand;
import org.firstinspires.ftc.teamcode.commands.ClawCommand;
import org.firstinspires.ftc.teamcode.commands.GlobalArmPositionCommand;
import org.firstinspires.ftc.teamcode.commands.WristCommand;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.java_websocket.enums.ReadyState;

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
                "[20] (placeholder - unused)",
                "[21] (placeholder - unused)",
                "[22] (placeholder - unused)",
                "[23] (placeholder - unused)",
                "[24] (placeholder - unused)",
                "[25] (placeholder - unused)",
        */

        //Vertical Grab - Ready (only when in travel position)
        gamepadDriver.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                () -> CommandScheduler.getInstance().schedule(
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                    new ParallelCommandGroup(
                                            new ArmAngleCommand(Constants.Arm.Angle.ANGLE_POSITIONS[2]),
                                            new ArmWinchCommand(Constants.Arm.Winch.WINCH_POSITIONS[2]),
                                            new WristCommand(Constants.Arm.Wrist.WRIST_POSITIONS[2]),
                                            new ClawCommand(Constants.Arm.Claw.CLAW_POSITIONS[2])
                                    ),
                                    new GlobalArmPositionCommand(2)
                                ),
                                null,
                                () -> Globals.ARM_POSITION==1
                        ))
        );

        //Horizontal Grab - Ready (only when in travel position)
        gamepadDriver.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                () -> CommandScheduler.getInstance().schedule(
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new ParallelCommandGroup(
                                                new ArmAngleCommand(Constants.Arm.Angle.ANGLE_POSITIONS[4]),
                                                new ArmWinchCommand(Constants.Arm.Winch.WINCH_POSITIONS[4]),
                                                new WristCommand(Constants.Arm.Wrist.WRIST_POSITIONS[4]),
                                                new ClawCommand(Constants.Arm.Claw.CLAW_POSITIONS[4])
                                        ),
                                        new GlobalArmPositionCommand(4)
                                ),
                                null,
                                () -> Globals.ARM_POSITION==1
                        ))
        );

        //Wall Grab - Ready (only when in travel position)
        gamepadDriver.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                () -> CommandScheduler.getInstance().schedule(
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new ParallelCommandGroup(
                                                new ArmAngleCommand(Constants.Arm.Angle.ANGLE_POSITIONS[6]),
                                                new ArmWinchCommand(Constants.Arm.Winch.WINCH_POSITIONS[6]),
                                                new WristCommand(Constants.Arm.Wrist.WRIST_POSITIONS[6]),
                                                new ClawCommand(Constants.Arm.Claw.CLAW_POSITIONS[6])
                                        ),
                                        new GlobalArmPositionCommand(6)
                                ),
                                null,
                                () -> Globals.ARM_POSITION==1
                        ))
        );

        //Right Bumper Pressed - Successful vertical/horizontal/wall grab - Set to Travel position
        gamepadDriver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                () -> CommandScheduler.getInstance().schedule(
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new ParallelCommandGroup(
                                                new ArmAngleCommand(Constants.Arm.Angle.ANGLE_POSITIONS[1]),
                                                new ArmWinchCommand(Constants.Arm.Winch.WINCH_POSITIONS[1]),
                                                new WristCommand(Constants.Arm.Wrist.WRIST_POSITIONS[1]),
                                                new ClawCommand(Constants.Arm.Claw.CLAW_POSITIONS[1])
                                        ),
                                        new GlobalArmPositionCommand(1)
                                ),null, () -> (Globals.ARM_POSITION==3 || Globals.ARM_POSITION==5 || Globals.ARM_POSITION==8)
                        ))
        );

        //High Basket - Ready Position
        gamepadDriver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                () -> CommandScheduler.getInstance().schedule(
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new ParallelCommandGroup(
                                                new ArmAngleCommand(Constants.Arm.Angle.ANGLE_POSITIONS[9]),
                                                new ArmWinchCommand(Constants.Arm.Winch.WINCH_POSITIONS[9]),
                                                new WristCommand(Constants.Arm.Wrist.WRIST_POSITIONS[9]),
                                                new ClawCommand(Constants.Arm.Claw.CLAW_POSITIONS[9])
                                        ),
                                        new GlobalArmPositionCommand(9)
                                ),
                                null,
                                () -> Globals.ARM_POSITION==1
                        ))
        );

        //Low Basket - Ready Position
        gamepadDriver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                () -> CommandScheduler.getInstance().schedule(
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new ParallelCommandGroup(
                                                new ArmAngleCommand(Constants.Arm.Angle.ANGLE_POSITIONS[11]),
                                                new ArmWinchCommand(Constants.Arm.Winch.WINCH_POSITIONS[11]),
                                                new WristCommand(Constants.Arm.Wrist.WRIST_POSITIONS[11]),
                                                new ClawCommand(Constants.Arm.Claw.CLAW_POSITIONS[11])
                                        ),
                                        new GlobalArmPositionCommand(11)
                                ),
                                null,
                                () -> Globals.ARM_POSITION==1
                        ))
        );

        //Hang Specimen (High Chamber) - Ready Position
        gamepadDriver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                () -> CommandScheduler.getInstance().schedule(
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new ParallelCommandGroup(
                                                new ArmAngleCommand(Constants.Arm.Angle.ANGLE_POSITIONS[13]),
                                                new ArmWinchCommand(Constants.Arm.Winch.WINCH_POSITIONS[13]),
                                                new WristCommand(Constants.Arm.Wrist.WRIST_POSITIONS[13]),
                                                new ClawCommand(Constants.Arm.Claw.CLAW_POSITIONS[13])
                                        ),
                                        new GlobalArmPositionCommand(13)
                                ),
                                null,
                                () -> Globals.ARM_POSITION==1
                        ))
        );

        //Hang Specimen (Low Chamber) - Ready Position
        gamepadDriver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                () -> CommandScheduler.getInstance().schedule(
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new ParallelCommandGroup(
                                                new ArmAngleCommand(Constants.Arm.Angle.ANGLE_POSITIONS[16]),
                                                new ArmWinchCommand(Constants.Arm.Winch.WINCH_POSITIONS[16]),
                                                new WristCommand(Constants.Arm.Wrist.WRIST_POSITIONS[16]),
                                                new ClawCommand(Constants.Arm.Claw.CLAW_POSITIONS[16])
                                        ),
                                        new GlobalArmPositionCommand(16)
                                ),
                                null,
                                () -> Globals.ARM_POSITION==1
                        ))
        );

        //Climber Ready Position (only from travel)
        gamepadDriver.getGamepadButton(GamepadKeys.Button.START).whenPressed(
                () -> CommandScheduler.getInstance().schedule(
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                    new ParallelCommandGroup(
                                            new ArmAngleCommand(Constants.Arm.Angle.ANGLE_POSITIONS[19]),
                                            new ArmWinchCommand(Constants.Arm.Winch.WINCH_POSITIONS[19]),
                                            new WristCommand(Constants.Arm.Wrist.WRIST_POSITIONS[19]),
                                            new ClawCommand(Constants.Arm.Claw.CLAW_POSITIONS[19])
                                    ),
                                new GlobalArmPositionCommand(19)
                        ),null, () -> Globals.ARM_POSITION==1
                ))
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

        //Drivetrain
        drivetrain.driveRobotCentric(
                gamepadDriver.getLeftX(),
                gamepadDriver.getLeftY(),
                gamepadDriver.getRightX(),
                false
        );

        //Right Trigger Pressed -- Attempt Vertical Grab, Horizontal Grab, or Wall Grab (only when in a ready position)
        if(gamepadDriver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)>Constants.General.CONTROLLER_DEADBAND) {
            //Vertical Grab - Attempt
            if(Globals.ARM_POSITION==2) {


            //Horizontal Grab - Attempt
            } else if(Globals.ARM_POSITION==4) {


            //Wall Grab - Attempt
            } else if(Globals.ARM_POSITION==4) {


            }
        }

        //Left Trigger Pressed -- Failed Grab Attempt, Basket Drop, or Specimen Score (only when in ready position)
        if(gamepadDriver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)>Constants.General.CONTROLLER_DEADBAND) {
            //Vertical Grab Attempt Failed, return to ready position
            if(Globals.ARM_POSITION==3) {
                CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new ParallelCommandGroup(
                                    new ArmAngleCommand(Constants.Arm.Angle.ANGLE_POSITIONS[2]),
                                    new ArmWinchCommand(Constants.Arm.Winch.WINCH_POSITIONS[2]),
                                    new WristCommand(Constants.Arm.Wrist.WRIST_POSITIONS[2]),
                                    new ClawCommand(Constants.Arm.Claw.CLAW_POSITIONS[2])
                            ),
                            new GlobalArmPositionCommand(2)
                    )
                );

            //Horizontal Grab Attempt Failed, return to ready position
            } else if(Globals.ARM_POSITION==5) {
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
                new SequentialCommandGroup(
                        new ClawCommand(Constants.Arm.Claw.CLAW_POSITIONS[10]),
                        new GlobalArmPositionCommand(10).withTimeout(500),
                        new ParallelCommandGroup(
                                new ArmAngleCommand(Constants.Arm.Angle.ANGLE_POSITIONS[1]),
                                new ArmWinchCommand(Constants.Arm.Winch.WINCH_POSITIONS[1]),
                                new WristCommand(Constants.Arm.Wrist.WRIST_POSITIONS[1]),
                                new ClawCommand(Constants.Arm.Claw.CLAW_POSITIONS[1])
                        ),
                        new GlobalArmPositionCommand(1)
                );

            //Sample Drop - Low Basket
            } else if(Globals.ARM_POSITION==11) {
                new SequentialCommandGroup(
                        new ClawCommand(Constants.Arm.Claw.CLAW_POSITIONS[12]),
                        new GlobalArmPositionCommand(12).withTimeout(500),
                        new ParallelCommandGroup(
                                new ArmAngleCommand(Constants.Arm.Angle.ANGLE_POSITIONS[1]),
                                new ArmWinchCommand(Constants.Arm.Winch.WINCH_POSITIONS[1]),
                                new WristCommand(Constants.Arm.Wrist.WRIST_POSITIONS[1]),
                                new ClawCommand(Constants.Arm.Claw.CLAW_POSITIONS[1])
                        ),
                        new GlobalArmPositionCommand(1)
                );

            //Specimen Hang - High Chamber
            } else if(Globals.ARM_POSITION==13) {
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new ArmAngleCommand(Constants.Arm.Angle.ANGLE_POSITIONS[14]),
                                new ArmWinchCommand(Constants.Arm.Winch.WINCH_POSITIONS[14]),
                                new WristCommand(Constants.Arm.Wrist.WRIST_POSITIONS[14]),
                                new ClawCommand(Constants.Arm.Claw.CLAW_POSITIONS[14])
                        ),
                        new GlobalArmPositionCommand(14).withTimeout(500),
                        new ParallelCommandGroup(
                                new ArmAngleCommand(Constants.Arm.Angle.ANGLE_POSITIONS[15]),
                                new ArmWinchCommand(Constants.Arm.Winch.WINCH_POSITIONS[15]),
                                new WristCommand(Constants.Arm.Wrist.WRIST_POSITIONS[15]),
                                new ClawCommand(Constants.Arm.Claw.CLAW_POSITIONS[15])
                        ),
                        new GlobalArmPositionCommand(15).withTimeout(500),
                        new ParallelCommandGroup(
                                new ArmAngleCommand(Constants.Arm.Angle.ANGLE_POSITIONS[1]),
                                new ArmWinchCommand(Constants.Arm.Winch.WINCH_POSITIONS[1]),
                                new WristCommand(Constants.Arm.Wrist.WRIST_POSITIONS[1]),
                                new ClawCommand(Constants.Arm.Claw.CLAW_POSITIONS[1])
                        ),
                        new GlobalArmPositionCommand(1)
                );

            //Specimen Hang - Low Chamber
            } else if(Globals.ARM_POSITION==16) {
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new ArmAngleCommand(Constants.Arm.Angle.ANGLE_POSITIONS[17]),
                                new ArmWinchCommand(Constants.Arm.Winch.WINCH_POSITIONS[17]),
                                new WristCommand(Constants.Arm.Wrist.WRIST_POSITIONS[17]),
                                new ClawCommand(Constants.Arm.Claw.CLAW_POSITIONS[17])
                        ),
                        new GlobalArmPositionCommand(17).withTimeout(500),
                        new ParallelCommandGroup(
                                new ArmAngleCommand(Constants.Arm.Angle.ANGLE_POSITIONS[18]),
                                new ArmWinchCommand(Constants.Arm.Winch.WINCH_POSITIONS[18]),
                                new WristCommand(Constants.Arm.Wrist.WRIST_POSITIONS[18]),
                                new ClawCommand(Constants.Arm.Claw.CLAW_POSITIONS[18])
                        ),
                        new GlobalArmPositionCommand(18).withTimeout(500),
                        new ParallelCommandGroup(
                                new ArmAngleCommand(Constants.Arm.Angle.ANGLE_POSITIONS[1]),
                                new ArmWinchCommand(Constants.Arm.Winch.WINCH_POSITIONS[1]),
                                new WristCommand(Constants.Arm.Wrist.WRIST_POSITIONS[1]),
                                new ClawCommand(Constants.Arm.Claw.CLAW_POSITIONS[1])
                        ),
                        new GlobalArmPositionCommand(1)
                );
            }
        }

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        loopTime = loop;
        telemetry.update();
    }
}
