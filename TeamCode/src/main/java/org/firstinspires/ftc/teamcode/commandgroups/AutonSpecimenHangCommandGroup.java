package org.firstinspires.ftc.teamcode.commandgroups;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.ClawCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmAngleSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ArmWinchSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class AutonSpecimenHangCommandGroup extends SequentialCommandGroup {
    private final int positionNumber = 5; //Basket drop position

    public AutonSpecimenHangCommandGroup(ArmAngleSubsystem armAngle, ArmWinchSubsystem armWinch, WristSubsystem wrist, ClawSubsystem claw) {

        addCommands(
            //Hang Specimen
            new ArmPositionCommandGroup(armAngle,armWinch,wrist,null,positionNumber).withTimeout(800),
            //new WaitCommand(400),

            //Open claw and go back to travel position, with winch and wrist moving first
            new ParallelCommandGroup(
                new ClawCommand(claw,Constants.Arm.Claw.CLAW_OPEN),
                new ArmPositionCommandGroup(armAngle,0.3,armWinch,0.0,wrist,0.0,null,0.0,1)
            )
        );

        addRequirements(armAngle,armWinch,wrist,claw);
    }
}
