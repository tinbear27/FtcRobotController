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

public class AutonBasketReadyCommandGroup  extends ParallelCommandGroup {

    private final int positionNumber=9; //Basket ready position

    //Moves all arm subsystems to drop position
    public AutonBasketReadyCommandGroup(ArmAngleSubsystem armAngle, ArmWinchSubsystem armWinch, WristSubsystem wrist, ClawSubsystem claw) {
        if(armAngle!=null) { addCommands(new ArmAngleCommand(armAngle, Constants.Arm.Angle.ANGLE_POSITIONS[positionNumber])); }
        if(armWinch!=null) { addCommands(new ArmWinchCommand(armWinch,Constants.Arm.Winch.WINCH_POSITIONS[positionNumber])); }
        if(wrist!=null) { addCommands(new WristCommand(wrist,Constants.Arm.Wrist.WRIST_POSITIONS[positionNumber])); }
        if(claw!=null) { addCommands(new ClawCommand(claw,Constants.Arm.Claw.CLAW_POSITIONS[positionNumber])); }
        addCommands(new InstantCommand(() -> RobotHardware.getInstance().setArmPosition(positionNumber)));

        addRequirements(armAngle,armWinch,wrist,claw);
    }
}
