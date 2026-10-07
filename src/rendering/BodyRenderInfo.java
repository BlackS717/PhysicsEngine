package rendering;

import java.awt.Color;

public class BodyRenderInfo {
    private Color color = Color.WHITE;
    private float radius = 1.0f;


    public BodyRenderInfo(){

    }

    public void setColor(Color color){
        this.color = color;
    }

    public Color getColor(){
        return this.color;
    }

    public void setRadius(float radius){
        this.radius = radius;
    }

    public float getRadius(){
        return this.radius;
    }
}
