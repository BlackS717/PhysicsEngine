package rendering;

import java.awt.Color;

public class BodyRenderInfo {
    private Color color = Color.WHITE;

    public BodyRenderInfo(){

    }

    public void setColor(Color color){
        this.color = color;
    }

    public Color getColor(){
        return this.color;
    }
}
