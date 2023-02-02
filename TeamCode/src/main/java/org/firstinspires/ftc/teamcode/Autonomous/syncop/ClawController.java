/**
 *      Syncable claw controller - designed to be wrapped by SyncAutoOp
**/

package org.firstinspires.ftc.teamcode.Autonomous.syncop;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ClawController {
    // motor
    private DcMotor claw = null;

    // time based claw control (at times)... because currently we have no time
    protected ElapsedTime runtime = new ElapsedTime();

    // external reference to opModeActive - probably a cleaner way to do this...
    Function<boolean, boolean> op_mode_is_active_pointer;

    // external logging
    protected Telemetry telemetry = null;
    
    public ClawController(HardwareMap hardware_map, Telemetry telemetry, Function<boolean, boolean> op_mode_is_active_pointer){
        // get lift hardware
        claw = hardwareMap.get(DcMotor.class, "claw");

        // set mode
        claw.setDirection(DcMotor.Direction.FORWARD);
        claw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        claw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // set initial position
        claw.setTargetPosition(0);
        claw.setPower(0.1);

        // save telemetry and opmode pointer
        this.telemetry = telemetry;
        this.op_mode_is_active_pointer = op_mode_is_active_pointer;
    }

    private boolean op_mode_is_active(){
        return op_mode_is_active_pointer.apply(true); // note input should not matter
    }
    
    public boolean busy(){
        return false;
    }

    public void grasp(){
        claw.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        claw.setPower(-0.35);
    }

    public void open(boolean wide){
        int pos = 5;

        pos += wide ? 5 : 0;
        
        claw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        claw.setTargetPosition(pos);
        claw.setPower(0.5);
    }
}