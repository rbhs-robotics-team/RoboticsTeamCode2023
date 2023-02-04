/**
 *      Syncable claw controller - designed to be wrapped by SyncAutoOp
**/

package org.firstinspires.ftc.teamcode.Autonomous.syncop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.Function;

public class ClawController {
    // motor
    private DcMotor claw = null;

    // time based claw control (at times)... because currently we have no time
    protected ElapsedTime runtime = new ElapsedTime();

    // external reference to opModeActive - probably a cleaner way to do this...
    private Function<Boolean, Boolean> op_mode_is_active_pointer;

    // external logging
    protected Telemetry telemetry = null;

    // internal state
    private boolean is_open = false;
    
    public ClawController(HardwareMap hardware_map, LinearOpMode opMode, Function<Boolean, Boolean> op_mode_is_active_pointer){
        // get lift hardware
        claw = hardware_map.get(DcMotor.class, "claw");

        // set mode
        claw.setDirection(DcMotor.Direction.FORWARD);
        claw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        claw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // set initial position
        claw.setTargetPosition(0);
        claw.setPower(0.3);

        claw.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // save telemetry and opmode pointer
        this.telemetry = opMode.telemetry;
        this.op_mode_is_active_pointer = op_mode_is_active_pointer;
    }

    private boolean op_mode_is_active(){
        return op_mode_is_active_pointer.apply(true); // note input should not matter
    }
    
    public boolean busy(){
        return is_open && claw.isBusy();
    }

    public void grasp(){
        is_open = false;

        claw.setTargetPosition(-60);
        claw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void open(boolean wide){
        is_open = true;

        claw.setTargetPosition(-80);
        claw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}