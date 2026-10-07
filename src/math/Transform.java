package math;

public class Transform {
    private Vector2 position;
    private Vector2 rotation;
    private int z;

    public Transform() {
        this(new Vector2(0, 0));
    }

    public Transform(Vector2 position) {
        this(position, new Vector2(0, 0));
    }

    public Transform(Transform other) {
        this(other.position, other.rotation, other.z);
    }

    public Transform(Vector2 position, Vector2 rotation) {
        this(position, rotation, 0);
    }

    public Transform(Vector2 position, Vector2 rotation, int z) {
        this.position = position;
        this.rotation = rotation;
        this.z = z;
    }

    public void setPosition(Vector2 position) {
        this.position = position;
    }

    public void setRotation(Vector2 rotation) {
        this.rotation = rotation;
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

    public int getZ() {
        return z;
    }

    @Override
    public String toString() {
        return "Transform{" +
                "position=" + position +
                ", rotation=" + rotation +
                ", z=" + z +
                '}';
    }
}
