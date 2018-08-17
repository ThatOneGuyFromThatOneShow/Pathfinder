package main.nodemaps;

/**
 * An ObjectMap is a map where you define entire object instead of individual nodes.
 */
public abstract class ObjectMap implements NodeMap {

    /**
     * Creates a new empty ObjectMap.
     *
     * @param width The width of the map.
     * @param height The height of the map.
     */
    public ObjectMap(int width, int height) {

    }

    /**
     * Adds a rectangle to the map. (x, y) being the top left corner.
     *
     * @param x The x location.
     * @param y The y location.
     * @param w The width of the rectangle.
     * @param h The height of the rectangle.
     */
    public abstract void addRectangle(int x, int y, int w, int h);
}
