package org.firstinspires.ftc.teamcode.commandgroups;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.ArmAngleCommand;
import org.firstinspires.ftc.teamcode.commands.ArmWinchCommand;
import org.firstinspires.ftc.teamcode.commands.ClawCommand;
import org.firstinspires.ftc.teamcode.commands.WristCommand;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.ArmAngleSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ArmWinchSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class ArmPositionCommandGroup extends ParallelCommandGroup {

    //Moves all arm subsystems to individual targets
    public ArmPositionCommandGroup(ArmAngleSubsystem armAngle, ArmWinchSubsystem armWinch, WristSubsystem wrist, ClawSubsystem claw, int positionNumber) {
        if(armAngle!=null) { addCommands(new ArmAngleCommand(armAngle, Constants.Arm.Angle.ANGLE_POSITIONS[positionNumber])); }
        if(armWinch!=null) { addCommands(new ArmWinchCommand(armWinch,Constants.Arm.Winch.WINCH_POSITIONS[positionNumber])); }
        if(wrist!=null) { addCommands(new WristCommand(wrist,Constants.Arm.Wrist.WRIST_POSITIONS[positionNumber])); }
        if(claw!=null) { addCommands(new ClawCommand(claw,Constants.Arm.Claw.CLAW_POSITIONS[positionNumber])); }
        addCommands(new InstantCommand(() -> RobotHardware.getInstance().setArmPosition(positionNumber)));

        addRequirements(armAngle,armWinch,wrist,claw);
    }

    //Moves all arm subsystems to individual targets, with delays per subsystem
    public ArmPositionCommandGroup(ArmAngleSubsystem armAngle, double armAngleDelay, ArmWinchSubsystem armWinch, double armWinchDelay, WristSubsystem wrist, double wristDelay, ClawSubsystem claw, double clawDelay, int positionNumber) {
        if(armAngle!=null) {
            if(armAngleDelay>0) {
                addCommands(new ArmAngleDelayCommandGroup(armAngle,Constants.Arm.Angle.ANGLE_POSITIONS[positionNumber],armAngleDelay));
            } else {
                addCommands(new ArmAngleCommand(armAngle, Constants.Arm.Angle.ANGLE_POSITIONS[positionNumber]));
            }
        }

        if(armWinch!=null) {
            if(armWinchDelay>0) {
                addCommands(new ArmWinchDelayCommandGroup(armWinch,Constants.Arm.Winch.WINCH_POSITIONS[positionNumber],armWinchDelay));
            } else {
                addCommands(new ArmWinchCommand(armWinch,Constants.Arm.Winch.WINCH_POSITIONS[positionNumber]));
            }
        }

        if(wrist!=null) {
            if(wristDelay>0) {
                addCommands(new WristDelayCommandGroup(wrist,Constants.Arm.Wrist.WRIST_POSITIONS[positionNumber],wristDelay));
            } else {
                addCommands(new WristCommand(wrist,Constants.Arm.Wrist.WRIST_POSITIONS[positionNumber]));
            }
        }

        if(claw!=null) {
            if(clawDelay>0) {
                addCommands(new ClawDelayCommandGroup(claw,Constants.Arm.Claw.CLAW_POSITIONS[positionNumber],clawDelay));
            } else {
                addCommands(new ClawCommand(claw,Constants.Arm.Claw.CLAW_POSITIONS[positionNumber]));
            }
        }

        addCommands(new InstantCommand(() -> RobotHardware.getInstance().setArmPosition(positionNumber)));

        addRequirements(armAngle,armWinch,wrist,claw);
    }

    //Moves all arm subsystems to individual targets, with delays per subsystem
    public ArmPositionCommandGroup(ArmAngleSubsystem armAngle, double armAngleDelay, long armAngleTimeout, ArmWinchSubsystem armWinch, double armWinchDelay, long armWinchTimeout, WristSubsystem wrist, double wristDelay, ClawSubsystem claw, double clawDelay, int positionNumber) {
        if(armAngle!=null) {
            if(armAngleDelay>0) {
                if(armAngleTimeout>0) {
                    addCommands(new ArmAngleDelayCommandGroup(armAngle, Constants.Arm.Angle.ANGLE_POSITIONS[positionNumber], armAngleDelay).withTimeout(armAngleTimeout));
                } else {
                    addCommands(new ArmAngleDelayCommandGroup(armAngle, Constants.Arm.Angle.ANGLE_POSITIONS[positionNumber], armAngleDelay));
                }
            } else {
                if(armAngleTimeout>0) {
                    addCommands(new ArmAngleCommand(armAngle, Constants.Arm.Angle.ANGLE_POSITIONS[positionNumber]).withTimeout(armAngleTimeout));
                } else {
                    addCommands(new ArmAngleCommand(armAngle, Constants.Arm.Angle.ANGLE_POSITIONS[positionNumber]));
                }
            }
        }

        if(armWinch!=null) {
            if(armWinchDelay>0) {
                if(armWinchTimeout>0) {
                    addCommands(new ArmWinchDelayCommandGroup(armWinch, Constants.Arm.Winch.WINCH_POSITIONS[positionNumber], armWinchDelay).withTimeout(armWinchTimeout));
                } else {
                    addCommands(new ArmWinchDelayCommandGroup(armWinch, Constants.Arm.Winch.WINCH_POSITIONS[positionNumber], armWinchDelay));
                }
            } else {
               if(armWinchTimeout>0) {
                  addCommands(new ArmWinchCommand(armWinch, Constants.Arm.Winch.WINCH_POSITIONS[positionNumber]).withTimeout(armWinchTimeout));
               } else {
                  addCommands(new ArmWinchCommand(armWinch, Constants.Arm.Winch.WINCH_POSITIONS[positionNumber]));
               }
            }
        }

        if(wrist!=null) {
            if(wristDelay>0) {
                addCommands(new WristDelayCommandGroup(wrist,Constants.Arm.Wrist.WRIST_POSITIONS[positionNumber],wristDelay));
            } else {
                addCommands(new WristCommand(wrist,Constants.Arm.Wrist.WRIST_POSITIONS[positionNumber]));
            }
        }

        if(claw!=null) {
            if(clawDelay>0) {
                addCommands(new ClawDelayCommandGroup(claw,Constants.Arm.Claw.CLAW_POSITIONS[positionNumber],clawDelay));
            } else {
                addCommands(new ClawCommand(claw,Constants.Arm.Claw.CLAW_POSITIONS[positionNumber]));
            }
        }

        addCommands(new InstantCommand(() -> RobotHardware.getInstance().setArmPosition(positionNumber)));

        addRequirements(armAngle,armWinch,wrist,claw);
    }
}
