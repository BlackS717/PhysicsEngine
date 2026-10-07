package physics;

public class CircleCollider extends  Collider{
    private float radius;

    public CircleCollider(Body body, float radius) {
        super(body, radius * 2, radius * 2);
        this.radius = radius;
    }

    public float getRadius() {
        return radius;
    }

    public void setRadius(float radius) {
        this.radius = radius;
        super.setWidth(radius * 2);
        super.setHeight(radius * 2);
    }
}
