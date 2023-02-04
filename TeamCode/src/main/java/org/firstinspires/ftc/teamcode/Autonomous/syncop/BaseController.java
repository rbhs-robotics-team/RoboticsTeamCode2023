/**
 *      Syncable base controller - designed to be wrapped by SyncAutoOp
**/

package org.firstinspires.ftc.teamcode.Autonomous.syncop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.robotcore.hardware.Gyroscope;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.Function;

public class BaseController {
    // motors
    protected DcMotor left_front = null;
    protected DcMotor left_back = null;
    protected DcMotor right_front = null;
    protected DcMotor right_back = null;

    // gyroscope vars
    protected BNO055IMU imu = null;
    protected double zero_heading = 0.0;
    private Orientation angles = null;
    private double margin = 1; // Margin of error for gyrometer based rotation
    protected double wheelPower = 0.8; // Wheel speed used for gyrometer based rotation

    // runtime (used for rotation)
    protected ElapsedTime runtime = new ElapsedTime();

    // external logging
    protected Telemetry telemetry = null;

    // external reference to opModeActive - there probably is a cleaner way to do this...
    private Function<Boolean, Boolean> op_mode_is_active_pointer;

    //constants (some public to facilitate native use of 'move' function)
    private double ticks_per_tile = 1120 / 4 / Math.PI * 24 * 1.0; // 1120 ticks/rev * 1/4π rev/in * 24 in/tile * gear_ratio
    
    public double forward_backward_tile_mod = 0.49;
    public double strafe_tile_mod = 0.58;

    public double default_power = 1.0;

    public BaseController (HardwareMap hardware_map, LinearOpMode opMode, Function<Boolean, Boolean> op_mode_is_active_pointer) {
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

        // gyroscope initialization
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.mode = BNO055IMU.SensorMode.IMU;
        params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        params.loggingEnabled = false;

        imu = hardware_map.get(BNO055IMU.class, "imu");
        imu.initialize(params);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        
        zero_heading = angles.firstAngle;

        // save telemetry and opmode
        this.telemetry = opMode.telemetry;
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

    public void set_left_wheel_power(double power){
        left_front.setPower(power);
        left_back.setPower(power);
    }

    public void set_right_wheel_power(double spd){
        right_front.setPower(power);
        right_back.setPower(power);
    }

    public void pause(double seconds){
        runtime.reset();
        while(op_mode_is_active() && (runtime.seconds() < seconds)){
            telemetry.addData("Waiting", "%2.5f / %2.5f", runtime.seconds(), seconds);
            telemetry.update();
        }
    }

    public boolean busy(){
        return left_front.isBusy() || right_front.isBusy() || left_back.isBusy() || right_back.isBusy();
    }

    private void sync(){
        while(op_mode_is_active() && busy()){}
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

    public void stopWheels(){
        set_left_wheel_power(0.0);
        set_right_wheel_power(0.0);
    }

    /** Gyrometer Access **/
    public double getAngle(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (angles.firstAngle - zero_heading + 360 + margin) % 360.0;
    }

    public double getNegAngle(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (angles.firstAngle - zero_heading - 720 - margin) % 360.0;
    }

    public double getSmAngle(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (angles.firstAngle - zero_heading + 360) % 360.0 - 180.0;
    }

    /** Gyroscope-Based Rotation **/
    public void left_gyro(double quarters){
        sync();
        set_wheel_mode(DcMotor.RunMode.RUN_USING_ENCODER);

        if(quarters > 2){
            left_gyro(quarters-2);
            quarters -= 2;
        }
        double a = 0.0;
        set_left_wheel_power(-wheelPower);
        set_right_wheel_power(wheelPower);
        runtime.reset();
        while(op_mode_is_active() && ((a=getAngle()) > 90*quarters+margin)){
            telemetry.addData("Path", "Turning Left: %2.5f S Elapsed, %2.3f deg", runtime.seconds(), a);
            telemetry.update();
        }
        pause(0.05);
        while(op_mode_is_active() && ((a=getAngle()) < 90*quarters+margin)){
            telemetry.addData("Path", "Turning Left: %2.5f S Elapsed, %2.3f deg", runtime.seconds(), a);
            telemetry.update();
        }
        set_left_wheel_power(wheelPower*0.2);
        set_right_wheel_power(-wheelPower*0.2);
        while(op_mode_is_active() && ((a=getAngle()) > 90*quarters+margin)){
            telemetry.addData("Path", "Turning Left: %2.5f S Elapsed, %2.3f deg", runtime.seconds(), a);
            telemetry.update();
        }
        stopWheels();
        runtime.reset();
        zero_heading = (zero_heading + 90.0) % 360.0;
    }

    public void right_gyro(double quarters){
        sync();
        set_wheel_mode(DcMotor.RunMode.RUN_USING_ENCODER);

        if(quarters > 2){
            right_gyro(quarters-2);
            quarters -= 2;
        }

        double a = 0.0;
        set_left_wheel_power(wheelPower);
        set_right_wheel_power(-wheelPower);
        runtime.reset();
        
        while(op_mode_is_active() && ((a=getNegAngle()%360) < -90*quarters-margin)){
            telemetry.addData("Path", "Turning Right: %2.5f S Elapsed, %2.3f deg", runtime.seconds(), a);
            telemetry.update();
        }

        pause(0.05);

        while(op_mode_is_active() && ((a=getNegAngle()) > -90*quarters-margin)){
            telemetry.addData("Path", "Turning Right: %2.5f S Elapsed, %2.3f deg", runtime.seconds(), a);
            telemetry.update();
        }
        
        set_left_wheel_power(-wheelPower*0.2);
        set_right_wheel_power(wheelPower*0.2);
        
        while(op_mode_is_active() && ((a=getNegAngle()) < -90*quarters-margin)){
            telemetry.addData("Path", "Turning Right: %2.5f S Elapsed, %2.3f deg", runtime.seconds(), a);
            telemetry.update();
        }
        
        stopWheels();
        runtime.reset();
        zero_heading = (zero_heading - 90.0) % 360.0;
    }

    public void turnZero(double margin, String name){
        double a = getSmAngle();
        runtime.reset();
        while(Math.abs(a) > margin){
            set_left_wheel_power(a*0.005);
            set_right_wheel_power(-a*0.005);
            telemetry.addData("Path", "%s: %2.5f S Elapsed, %2.3", name, a, runtime.seconds());
            telemetry.update();
            a = getSmAngle();
        }
        stopWheels();
    }
    
    public void turnZero(double margin){
        turnZero(margin, "Turning");
    }

    public void turnZero(){
        turnZero(5.0);
    }
};