package physics;

import math.Vector2;

public class BoxCollider extends  Collider{
    public BoxCollider(Body body, float width, float height) {
        super(body, width, height);
    }

    public void setSize(float width, float height) {
        super.setWidth(width);
        super.setHeight(height);
    }

    public boolean contains(Vector2 point) {
        Vector2 center = getCenter();
        float halfWidth = getWidth() / 2;
        float halfHeight = getHeight() / 2;

        return point.getX() >= center.getX() - halfWidth &&
               point.getX() <= center.getX() + halfWidth &&
               point.getY() >= center.getY() - halfHeight &&
               point.getY() <= center.getY() + halfHeight;
    }
}
