/**
 *      Syncable slide controller - designed to be wrapped by SyncAutoOp
**/

package org.firstinspires.ftc.teamcode.Autonomous.syncop;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SlideController {
    // motor
    protected DcMotor lift = null;

    // external logging
    protected Telemetry telemetry = null;

    public SlideController(HardwareMap hardware_map, Telemetry telemetry){
        // get lift hardware
        lift = hardwareMap.get(DcMotor.class, "lift");

        // set lift mode
        lift.setDirection(DcMotor.Direction.FORWARD);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // save telemetry reference
        this.telemetry = telemetry;
    }
}