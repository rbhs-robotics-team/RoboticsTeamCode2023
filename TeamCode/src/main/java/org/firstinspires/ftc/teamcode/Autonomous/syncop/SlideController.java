/**
 *      Syncable slide controller - designed to be wrapped by SyncAutoOp
**/

package org.firstinspires.ftc.teamcode.Autonomous.syncop;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.Function;

public class SlideController {
    // motor
    protected DcMotor lift = null;

    // external logging
    protected Telemetry telemetry = null;

    public SlideController(HardwareMap hardware_map, Telemetry telemetry, Function<Boolean, Boolean> op_mode_is_active_pointer){
        // get lift hardware
        lift = hardware_map.get(DcMotor.class, "lift");

        // set lift mode
        lift.setDirection(DcMotor.Direction.FORWARD);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // save telemetry reference
        this.telemetry = telemetry;
        
        // opmode_is_active_pointer ignored
    }

    // sync
    public boolean busy(){
        return lift.isBusy();
    }
    
    // simplified motor interface
    public void lift(String position, boolean drop){
        int pos = lift.getCurrentPosition();

        switch(position){
            case "max": pos = 3200; break;
            case "high": pos = 3000; break;
            case "medium": pos = 2150; break;
            case "low": pos = 1300; break;
            case "ground":
            case "min":
                pos = 0; break;
        }

        pos -= drop ? 400 : 0;
        pos = (pos < 0) ? 0 : pos;

        lift.setTargetPosition(pos);
        lift.setPower(0.5);
    }
}