package math;

public class Transform {
    private Vector2 position;
    private float rotation; // radians, counter-clockwise (y up), 0 = pointing along +x
    private int z;

    public Transform() {
        this(new Vector2(0, 0));
    }

    public Transform(Vector2 position) {
        this(position, 0f);
    }

    /** Deep copy, so the copy never shares a mutable Vector2 with the original. */
    public Transform(Transform other) {
        this(new Vector2(other.position), other.rotation, other.z);
    }

    public Transform(Vector2 position, float rotation) {
        this(position, rotation, 0);
    }

    public Transform(Vector2 position, float rotation, int z) {
        this.position = position;
        this.rotation = rotation;
        this.z = z;
    }

    public void setPosition(Vector2 position) {
        this.position = position;
    }

    public void setRotation(float rotation) {
        this.rotation = rotation;
    }

    /** Adds to the current rotation (radians). */
    public void rotate(float deltaRadians) {
        this.rotation += deltaRadians;
    }

    public void setZ(int z) {
        this.z = z;
    }

    public Vector2 getPosition() {
        return position;
    }

    public float getRotation() {
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