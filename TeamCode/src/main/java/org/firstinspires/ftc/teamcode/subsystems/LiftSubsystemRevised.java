package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class LiftSubsystemRevised extends SubsystemBase {
    private CRServo lift;
    private final Telemetry thisTelemetry;

    private double raiseTime=6.0; //Seconds to lift to full position
    private double lowerTime=6.0; //Seconds to lower to original position
    private double targetTime=0.0; //Time until power down
    private ElapsedTime elapsedTime = new ElapsedTime();

    public LiftSubsystemRevised(final HardwareMap hMap, Telemetry telemetry) {
        this.lift=hMap.get(CRServo.class, Constants.Lift.LIFT_ID);
        lift.setDirection(Constants.Lift.LIFT_DIRECTION);
        this.thisTelemetry=telemetry;
    }

    public void raise() {
        lift.setPower(Constants.Lift.LIFT_POWER);
        targetTime=elapsedTime.time()+raiseTime;
    }

    public void lower() {
        lift.setPower(-Constants.Lift.LIFT_POWER);
        targetTime=elapsedTime.time()+lowerTime;
    }

    public boolean reachedTarget() {
        return (targetTime==0.0);
    }

    @Override
    public void periodic() {
        //check to see if target time reached
        if(targetTime>0.0 && elapsedTime.time()>targetTime) {
            lift.setPower(0.0);
            targetTime=0.0;
        }
    }
}
