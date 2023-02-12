package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.syncop.SyncError;
import org.firstinspires.ftc.teamcode.Autonomous.syncop.SyncStopped;

@Autonomous(name="TurningTest", group="Test")
public class TurningTest extends SyncAutoOp2023 {

    @Override
    public void runOpMode() {
        initialize(hardwareMap, telemetry);

        waitForStart();

        try {
            turn(1.0);
            turn(3.0);

            sync();
        } catch(SyncError e) {
            telemetry_.addData("Path", e.getMessage());
            telemetry_.update();
        } catch(SyncStopped e){
            telemetry_.addData("Path", e.getMessage());
            telemetry_.update();
        } finally {
            shutdown();
        }
    }
}