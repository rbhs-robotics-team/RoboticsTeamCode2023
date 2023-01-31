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

@Autonomous(name="Route", group="Test")
public class Route extends SyncAutoOp2023 {

    private Camera camera;
    
    @Override public void runOpMode() throws InterruptedException {
        initialize(hardwareMap, telemetry);

        camera = new Camera("Webcam 1");

        camera.capture();

        waitForStart();
        
        forward(1.0, 0.2);
        
        sync();
    }
}