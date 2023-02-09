package org.firstinspires.ftc.teamcode.Autonomous.old;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.old.AutoOp2023;

@Autonomous(name="Test", group="Test")

public class Test extends AutoOp2023 {
    
    @Override
    public void runOpMode() {
        initialize(hardwareMap, telemetry);
        waitForStart();
        forward(1, 0.2);
        strafeLeft(1, 0.2);
        backward(1, 0.2);
        strafeRight(1, 0.2);
        left(1);
        right(1);
        // test
    }
}