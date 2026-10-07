package physics;

import math.Vector2;

public class Collider {
    private Body body;

    private float width, height;
    private Vector2 offset = new Vector2(0, 0);

    public Collider(Body body, float width, float height) {
        this.body = body;
        this.width = width;
        this.height = height;
    }

    public float getWidth() {
        return width;
    }

    protected void setWidth(float width) {
        this.width = width;
    }

    public float getHeight() {
        return height;
    }

    protected void setHeight(float height) {
        this.height = height;
    }

    public Vector2 getOffset() {
        return offset;
    }

    public void setOffset(Vector2 offset) {
        this.offset = offset;
    }

    public Vector2 getCenter(){
        return body.getTransform().getPosition().add(offset);
    }

    public Body getBody() {
        return body;
    }
}
