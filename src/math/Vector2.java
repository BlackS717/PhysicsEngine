package math;

public class Vector2 {
    private float x, y;

    public Vector2(float x, float y) {
        this.x = x;
        this.y = y;
    }

    public Vector2(Vector2 other) {
        this(other.getX(), other.getY());
    }

    public float getX() {
        return x;
    }

    public float getY() {
        return y;
    }

    public void setX(float x){
        this.x = x;
    }

    public void setY(float y){
        this.y = y;
    }

    public void setCoordinates(float x, float y){
        setX(x);
        setY(y);
    }

    public Vector2 add(Vector2 v) {
        return new Vector2(x + v.getX(), y + v.getY());
    }

    public Vector2 sub(Vector2 v) {
        return new Vector2(x - v.getX(), y - v.getY());
    }

    public Vector2 mult(float f) {
        return new Vector2(x * f, y * f);
    }

    public Vector2 mult(Vector2 v) {
        return new Vector2(x * v.getX(), y * v.getY());
    }

    public Vector2 div(float f) {
        return new Vector2(x / f, y / f);
    }

    public Vector2 div(Vector2 v) {
        return new Vector2(x / v.getX(), y / v.getY());
    }

    public float dotProduct(Vector2 v) {
        return x * v.getX() + y * v.getY();
    }

    public float magnitude() {
        return (float) Math.sqrt(x * x + y * y);
    }
}
