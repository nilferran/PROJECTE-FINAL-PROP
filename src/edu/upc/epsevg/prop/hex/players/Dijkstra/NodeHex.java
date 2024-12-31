/**
 * Clase que representa un nodo en un grafo utilizado para el juego Hex.
 * Un nodo está asociado con una posición en el tablero y tiene conexiones
 * con nodos adyacentes que se representan mediante un mapa de distancias.
 */


/**
 *@author JuanNil
 */

package edu.upc.epsevg.prop.hex.players.Dijkstra;

import java.awt.Point;
import java.util.*;

public class NodeHex {

    /**
     * La posición del nodo en el tablero.
     */
    private Point point;

    /**
     * La ruta más corta desde el origen hasta este nodo.
     */
    private List<NodeHex> shortestPath = new LinkedList<>();

    /**
     * La distancia mínima calculada desde el nodo de origen.
     * Por defecto es {@link Integer#MAX_VALUE}.
     */
    private Integer distance = Integer.MAX_VALUE;

    /**
     * Nodos adyacentes y sus distancias asociadas desde este nodo.
     */
    private Map<NodeHex, Integer> adjacentNodeHexs = new HashMap<>();

    /**
     * Constructor que inicializa un nodo con una posición específica.
     *
     * @param point La posición del nodo en el tablero.
     */
    public NodeHex(Point point) {
        this.point = point;
    }

    /**
     * Agrega una conexión desde este nodo a otro nodo con una distancia específica.
     *
     * @param destination El nodo destino.
     * @param distance    La distancia hasta el nodo destino.
     */
    public void addDestination(NodeHex destination, int distance) {
        adjacentNodeHexs.put(destination, distance);
    }

    /**
     * Obtiene la posición del nodo.
     *
     * @return La posición del nodo.
     */
    public Point getPoint() {
        return point;
    }

    /**
     * Establece la posición del nodo.
     *
     * @param point La nueva posición del nodo.
     */
    public void setPoint(Point point) {
        this.point = point;
    }

    /**
     * Obtiene la ruta más corta desde el origen hasta este nodo.
     *
     * @return La ruta más corta.
     */
    public List<NodeHex> getShortestPath() {
        return shortestPath;
    }

    /**
     * Establece la ruta más corta hasta este nodo.
     *
     * @param shortestPath La nueva ruta más corta.
     */
    public void setShortestPath(List<NodeHex> shortestPath) {
        this.shortestPath = shortestPath;
    }

    /**
     * Obtiene la distancia mínima desde el nodo de origen.
     *
     * @return La distancia mínima.
     */
    public Integer getDistance() {
        return distance;
    }

    /**
     * Establece la distancia mínima desde el nodo de origen.
     *
     * @param distance La nueva distancia mínima.
     */
    public void setDistance(Integer distance) {
        this.distance = distance;
    }

    /**
     * Obtiene los nodos adyacentes y sus distancias asociadas.
     *
     * @return Un mapa de nodos adyacentes y distancias.
     */
    public Map<NodeHex, Integer> getAdjacentNodeHexs() {
        return adjacentNodeHexs;
    }

    /**
     * Establece los nodos adyacentes y sus distancias asociadas.
     *
     * @param adjacentNodeHexs Un mapa de nodos adyacentes y distancias.
     */
    public void setAdjacentNodeHexs(Map<NodeHex, Integer> adjacentNodeHexs) {
        this.adjacentNodeHexs = adjacentNodeHexs;
    }

    /**
     * Compara este nodo con otro objeto para verificar igualdad.
     * Dos nodos son iguales si tienen la misma posición.
     *
     * @param o El objeto a comparar.
     * @return {@code true} si los nodos son iguales, de lo contrario {@code false}.
     */
    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof NodeHex)) return false;
        NodeHex node = (NodeHex) o;
        return Objects.equals(getPoint(), node.getPoint());
    }

    /**
     * Calcula el código hash del nodo basado en su posición.
     *
     * @return El código hash del nodo.
     */
    @Override
    public int hashCode() {
        return Objects.hash(getPoint());
    }
}
