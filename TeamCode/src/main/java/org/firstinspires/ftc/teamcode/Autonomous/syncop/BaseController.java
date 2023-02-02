/**
 *      Syncable base controller - designed to be wrapped by SyncAutoOp
**/

package org.firstinspires.ftc.teamcode.Autonomous.syncop;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.Function;

public class BaseController {
    // motors
    protected DcMotor left_front = null;
    protected DcMotor left_back = null;
    protected DcMotor right_front = null;
    protected DcMotor right_back = null;

    // external logging
    protected Telemetry telemetry = null;

    // external reference to opModeActive - probably a cleaner way to do this...
    private Function<Boolean, Boolean> op_mode_is_active_pointer;

    //constants (some public to facilitate native use of 'move' function)
    private double ticks_per_tile = 1120 / 4 / Math.PI * 24 * 1.0; // 1120 ticks/rev * 1/4π rev/in * 24 in/tile * gear_ratio
    
    public double forward_backward_tile_mod = 0.49;
    public double strafe_tile_mod = 0.58;

    public double default_power = 1.0;

    public BaseController(HardwareMap hardware_map, Telemetry telemetry, Function<Boolean, Boolean> op_mode_is_active_pointer) {
        // link to respective hardware bus
        left_front = hardware_map.get(DcMotor.class, "frontLeft");
        left_back = hardware_map.get(DcMotor.class, "backLeft");
        right_front = hardware_map.get(DcMotor.class, "frontRight");
        right_back = hardware_map.get(DcMotor.class, "backRight");

        // set mode
        left_front.setDirection(DcMotor.Direction.REVERSE);
        left_back.setDirection(DcMotor.Direction.REVERSE);
        right_front.setDirection(DcMotor.Direction.FORWARD);
        right_back.setDirection(DcMotor.Direction.FORWARD);

        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // save telemetry and opmode
        this.telemetry = telemetry;
        this.op_mode_is_active_pointer = op_mode_is_active_pointer;
    }

    private boolean op_mode_is_active(){
        return op_mode_is_active_pointer.apply(true); // note input should not matter
    }
    
    private void set_wheel_mode(DcMotor.RunMode mode){
        left_front.setMode(mode);
        left_back.setMode(mode);
        right_front.setMode(mode);
        right_back.setMode(mode);
    }

    private void set_wheel_power(double power){
        left_front.setPower(power);
        left_back.setPower(power);
        right_front.setPower(power);
        right_back.setPower(power);
    }

    public boolean busy(){
        return left_front.isBusy() || right_front.isBusy() || left_back.isBusy() || right_back.isBusy();
    }

    private void sync(){
        while(busy()){} // this should also consider 
    }
    
    public void move(double x_tiles, double y_tiles, double power){
        /** note that while it is possible to vastly increase complexity to avoid syncing here, for now it does not seem worth it **/
        sync(); // prevent multiple base commands from being executed out of order
        
        set_wheel_mode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left_front.setTargetPosition((int)((y_tiles + x_tiles) * ticks_per_tile));
        left_back.setTargetPosition((int)((y_tiles - x_tiles) * ticks_per_tile));
        right_front.setTargetPosition((int)((y_tiles - x_tiles) * ticks_per_tile));
        right_back.setTargetPosition((int)((y_tiles + x_tiles) * ticks_per_tile));
        
        set_wheel_power(power);

        set_wheel_mode(DcMotor.RunMode.RUN_TO_POSITION);
        
        /**                         WARNING: is this code needed?? if so, there may be some issues...
        leftfront.setPower(0.0);
        leftback.setPower(0.0);
        rightfront.setPower(0.0);
        rightback.setPower(0.0);
        
        set_wheel_mode(DcMotor.RunMode.RUN_USING_ENCODER); // clean up... maybe
        **/
    }

    public void move(double x_tiles, double y_tiles){
        move(x_tiles, y_tiles, default_power);
    }

    // simplified interface for the move function
    public void forward(double tiles, double power){
        move(0.0, tiles * forward_backward_tile_mod, power);
    }

    public void forward(double tiles){ forward(tiles, default_power); }

    public void backward(double tiles, double power){
        move(0.0, -tiles * forward_backward_tile_mod, power);
    }

    public void backward(double tiles){ backward(tiles, default_power); }

    public void strafe_right(double tiles, double power){
        move(tiles * strafe_tile_mod, 0.0, power);
    }

    public void strafe_right(double tiles){ strafe_right(tiles, default_power); }

    public void strafe_left(double tiles, double power){
        move(-tiles * strafe_tile_mod, 0.0, power);
    }

    public void strafe_left(double tiles){ strafe_left(tiles, default_power); }

    /** ! ADD TURNING CODE ! **/
};