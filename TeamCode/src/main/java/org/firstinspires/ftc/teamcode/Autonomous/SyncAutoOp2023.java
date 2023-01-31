/**
*       Non-blocking Hardware Interface
**/

package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.syncop.BaseController;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

public abstract class SyncAutoOp2023 extends LinearOpMode {
    private BaseController base = null;

    private Telemetry telemetry = null;

    public void initialize(HardwareMap hardware_map, Telemetry telemetry){
        this.telemetry = telemetry;
        
        base = new BaseController(hardware_map, telemetry);
    }

    // packaged functions
    public boolean busy(){
        return base.busy();
    }
    
    public void sync(){
        while(opModeIsActive() && busy()){
            telemetry.addData("Path", "Base{%s}", base.busy() ? "T" : "F");
            telemetry.update();
        }
    }

    // component functions
    public void forward(double tiles, double power){ base.forward(tiles, power); }

    public void forward(double tiles){ base.forward(tiles); }

    public void backward(double tiles, double power){ base.backward(tiles, power); }

    public void backward(double tiles){ base.backward(tiles); }

    public void strafe_right(double tiles, double power){ base.strafe_right(tiles, power); }

    public void strafe_right(double tiles){ base.strafe_right(tiles); }

    public void strafe_left(double tiles, double power){ base.strafe_left(tiles, power); }

    public void strafe_left(double tiles){ base.strafe_left(tiles); }
}