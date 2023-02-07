/**
 *  Things to consider
 *      a) The claw needs constant power the whole time (perhaps extract into a seperate class that runs in a seperate thread?)
 *      b) Moving and raising the claw can be done in parrellel (extract claw into a seperate, nonblocking, class - no extra thread needed)
 *      c) Perhaps straffing and moving forward can be done in parrellel, I am less sure about how to accomplish this
 *          - Similarly, it may be possible to simultaneously turn and move, more investigation would be needed
 *      d) With not insignificant work (perhaps it is too late in season) we could add a path finding algorithm
 *          - Allow for much easier autonomous coding
 *          - Allow for driver control shortcuts (press a button to go home, for example)
 *          - Allow for failure recovery (attempt to figure out where you are, perhaps by using the signal cone, then recover from there)
 *
 *          - Could sound cool if we explain how it works to the judges (which is perhaps our best shot at making state to be honest)
**/

package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="25pt: Right Placement", group="Autonomous")
public class RightRoute extends SyncAutoOp2023 {

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
        strafe_left(1, 0.2);
        sync();

        forward(1, 0.2);
        lift("max");
        sync();

        left(0.5);
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

        left(.5);
        sync();

        lift("min");
        if(position.equals(SleeveDetection.ParkingPosition.RIGHT)) {
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