package frc.robot.util;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class CommandXboxControllerExpand extends CommandXboxController {
    private final XboxController m_hid;

    public CommandXboxControllerExpand(int port) {
        super(port);
        this.m_hid = getHID();
    }
    
    /**
     * Get the POV value of the controller
     * @return  The POV value of the controller
     */
    public int getPOV() {
        return m_hid.getPOV();
    }

    /**
     * Gets the A B X Y Buttons Acting as POV
     * @return  The POV
     */
    public int getABXYasPOV() {
        int deg = -1;

        if(m_hid.getYButton() && m_hid.getBButton()) {
            deg = 45;
        } else if(m_hid.getBButton() && m_hid.getAButton()) {
            deg = 135;
        } else if(m_hid.getAButton() && m_hid.getXButton()) {
            deg = 225;
        } else if(m_hid.getXButton() && m_hid.getYButton()) {
            deg = 315;
        } else if(m_hid.getAButton()) {
            deg = 180;
        } else if(m_hid.getBButton()) {
            deg = 90;
        } else if(m_hid.getXButton()) {
            deg = 270;
        } else if(m_hid.getYButton()) {
            deg = 0;
        }

        return deg;
    }

    public boolean isABXYPressed() {
        return m_hid.getAButton() || m_hid.getBButton() || m_hid.getXButton() || m_hid.getYButton();
    }

    /**
     * Gets both ABXY POV and POV and mixes them
     * @return  All POV (0-315)
     */
    public int getMixPOV() {
        return isABXYPressed() ? getABXYasPOV() : getPOV();
    }
}
