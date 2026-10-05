package math;

public class Transform {
    private Vector2 position;
    private Vector2 rotation;
    private Vector2 scale;
    private int z;

    public Transform() {
        this(new Vector2(0, 0));
    }

    public Transform(Vector2 position) {
        this(position, new Vector2(0, 0));
    }

    public Transform(Vector2 position, Vector2 rotation) {
        this(position, rotation, new Vector2(1, 1), 0);
    }

    public Transform(Vector2 position, Vector2 rotation, Vector2 scale, int z) {
        this.position = position;
        this.scale = scale;
        this.rotation = rotation;
        this.z = z;
    }

    public void setPosition(Vector2 position) {
        this.position = position;
    }

    public void setRotation(Vector2 rotation) {
        this.rotation = rotation;
    }

    public void setScale(Vector2 scale) {
        this.scale = scale;
    }

    public void setZ(int z) {
        this.z = z;
    }

    public Vector2 getPosition() {
        return position;
    }

    public Vector2 getRotation() {
        return rotation;
    }

    public Vector2 getScale() {
        return scale;
    }

    public int getZ() {
        return z;
    }
}
