package org.firstinspires.ftc.teamcode.testing;


import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "OctoQuad Test", group="OctoQuad")
public class OctoQuadTest extends LinearOpMode {
    // Identify which encoder OctoQuad inputs are connected to each odometry pod.
    private final int OQ_ODO_RIGHT_PORT = 0; // Odometry Right-side Deadwheel OctoQuad port
    private final int OQ_ODO_MIDDLE_PORT = 1; // Odometry Middle-side Deadwheel OctoQuad port
    private final int OQ_ODO_LEFT_PORT = 2; // Odometry Left-side Deadwheel OctoQuad port

    private final String OCTOQUAD_ID = "OctoQuad";

    // Declare the OctoQuad object and members to store encoder positions and velocities
    private OctoQuad octoquad;

    @Override
    public void runOpMode() {
        // Connect to OctoQuad by referring to its name in the Robot Configuration.
        octoquad = hardwareMap.get(OctoQuad.class, OCTOQUAD_ID);

        waitForStart();

        // Set all the encoder inputs to zero.
        octoquad.resetAllPositions();

        while (opModeIsActive()) {
            int[] encoderValues = octoquad.readAllPositions();

            telemetry.addData("Right ", "%8d counts", encoderValues[OQ_ODO_RIGHT_PORT]);
            telemetry.addData("Middle", "%8d counts", encoderValues[OQ_ODO_MIDDLE_PORT]);
            telemetry.addData("Left ", "%8d counts", encoderValues[OQ_ODO_LEFT_PORT]);
            telemetry.update();
        }
    }
}