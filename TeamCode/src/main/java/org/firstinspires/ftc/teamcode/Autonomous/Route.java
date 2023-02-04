package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Route", group="Test")
public class Route extends SyncAutoOp2023 {

    private Camera camera;
    
    @Override public void runOpMode() throws InterruptedException {
        initialize(hardwareMap);

        camera = new Camera(hardwareMap,"Webcam 1");

        camera.capture();

        waitForStart();
        resetZeroHeading();

        try {
            grasp();
            sync();

            forward(0.2, 0.2);
            strafe_left(1, 0.2);
            sync();

            forward(1, 0.2);
            lift("max");
            sync();

            left(0.5);
            sync();

            forward(0.3, 0.2);
            sync();
            
            lift("max", true);
            sync();

            open();
            sync();
            pause(.5);
            lift("max");
            sync();

            backward(0.3, 0.2);
            sync();

            right(0.5);
            lift("min");
            sync();
        } catch(SyncStopped e){
            telemetry.addData("Path", "SyncStopped{%s}", e.getMessage());
        } catch(SyncError e) {
            telemetry.addData("Path", "SyncError{%s}", e.getMessage());
        } finally {
            telemetry.update();
            shutdown();
        }
    }
}