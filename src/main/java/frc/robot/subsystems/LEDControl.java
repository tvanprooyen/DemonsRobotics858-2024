package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDControl extends SubsystemBase {

    public enum AnimationType {
        RAINBOW,
        RGBFADE
    }

    private CANdle candle;
    private CANdleConfiguration config;
    private AnimationType animationSelected = null;
    private AnimationType prevAnimationSelected = null;
    
    public LEDControl() {

        candle = new CANdle(15, "rio");
        config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB;

        candle.configAllSettings(config);

        this.animationSelected = AnimationType.RAINBOW;
        this.prevAnimationSelected = AnimationType.RAINBOW;
        setLEDAnimation(AnimationType.RAINBOW);

    }


    private void setLEDAnimation(AnimationType aType) {
        Animation toAnimate = null;
        
        switch (aType) {
            case RAINBOW:
                toAnimate = new RainbowAnimation(1, 0.5, 82);
            break;
            
            case RGBFADE:
                setLEDRGB(255, 0, 255);
                toAnimate = new RgbFadeAnimation(1, 0.5, 82);
            break;
        }

        candle.animate(toAnimate);
    }

    public void setLEDRGB(int r, int g, int b) {
        candle.setLEDs(r, g, b);
    }

    /* public void setLEDTwinkle(int r, int g, int b){
        Animation animate = new TwinkleAnimation(r,g,b);
        candle.animate(animate);
    } */

    public void cancelLEDAnimation(){
        candle.animate(null);
    }

    private void setAnimationType(AnimationType aType) {
        this.animationSelected = aType;
    }


    /* @Override
    public void periodic() {
        if(this.animationSelected != this.prevAnimationSelected) {
            setLEDAnimation(animationSelected);
        }

        this.prevAnimationSelected = this.animationSelected;
    } */
}
