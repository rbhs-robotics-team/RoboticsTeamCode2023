package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="25pt: Left Placement", group="Autonomous")
public class LeftRoute extends SyncAutoOp2023 {

    private Camera camera;

    private SleeveDetection.ParkingPosition position = null;

    @Override public void runOpMode() throws InterruptedException {
        initialize(hardwareMap, telemetry);

        camera = new Camera(hardwareMap,"Webcam 1");

        while (!isStarted()) {
            camera.capture();
            telemetry_.addData("ROTATION: ", camera.read());
            telemetry_.update();
        }
        camera.capture();
        position = camera.read();

        waitForStart();
        resetZeroHeading();

        grasp();
        sync();

        forward(0.1, 0.2);
        strafe_right(1, 0.2);
        sync();

        forward(1, 0.2);
        lift("max");
        sync();

        right(0.5);
        sync();

        forward(0.35, 0.1);
        sync();
        lift("max", true);
        sync();

        open();
        sync();
        pause(.5);
        lift("max");
        sync();

        backward(0.35, 0.1);
        sync();

        right(.5);
        sync();

        lift("min");
        if(position.equals(SleeveDetection.ParkingPosition.LEFT)) {
            backward(2, 0.2);
        } else if(position.equals(SleeveDetection.ParkingPosition.CENTER)) {
            backward(1, 0.2);
        }
        sync();

        open();
        sync();

        stop();
    }
}