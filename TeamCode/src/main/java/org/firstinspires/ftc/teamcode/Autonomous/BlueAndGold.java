package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.syncop.SyncError;
import org.firstinspires.ftc.teamcode.Autonomous.syncop.SyncStopped;

@Autonomous(name="BlueAndGold", group="Test")
public class BlueAndGold extends SyncAutoOp2023 {

    @Override
    public void runOpMode() {
        initialize(hardwareMap, telemetry);

        waitForStart();

        try {
            // code
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