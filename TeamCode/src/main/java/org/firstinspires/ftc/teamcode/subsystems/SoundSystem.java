package org.firstinspires.ftc.teamcode.subsystems;

import android.media.MediaPlayer;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.R;

public class SoundTeleOP {
    public MediaPlayer DJ_Stalin;

    public SoundTeleOP(MediaPlayer m){
        this.DJ_Stalin = m;
    }

    public void c(Gamepad gp, HardwareMap hm, Telemetry t){
        if (gp.dpad_up){
            DJ_Stalin = null;
            DJ_Stalin =  MediaPlayer.create(hm.appContext, R.raw.gracious);
            t.addLine("Added 'Gracious Professionalism' to player");
            t.update();
        }
        if (gp.dpad_down) {
            DJ_Stalin = MediaPlayer.create(hm.appContext, R.raw.wegotem);
        }
        if (gp.dpad_left){
            DJ_Stalin.stop();
            t.addLine("Resetted DJ Stalin ''");
            t.update();
        }
    }
}
