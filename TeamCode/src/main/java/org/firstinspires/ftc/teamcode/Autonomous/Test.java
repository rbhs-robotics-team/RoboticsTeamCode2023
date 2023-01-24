package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Test", group="Test")

public class Test extends AutoOp2023 {

    @Override
    public void runOpMode() {
        initialize(hardwareMap, telemetry);
        waitForStart();
        forward(0.2, 0.2);
        strafeLeft(0.2, 0.2);
        backward(0.2, 0.2);
        strafeRight(0.2, 0.2);
        left(0.5);
        right(0.5);
        //test
    }
}