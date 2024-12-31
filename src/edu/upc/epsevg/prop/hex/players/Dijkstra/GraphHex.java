/**
 * Clase que representa un grafo de nodos Hex utilizado en el juego Hex.
 * El grafo contiene una lista de nodos y proporciona métodos para
 * gestionar y acceder a los mismos.
 */

/**
 *@author JuanNil
 */
 
package edu.upc.epsevg.prop.hex.players.Dijkstra;

import java.util.ArrayList;
import java.util.List;

public class GraphHex {

    /**
     * Lista de nodos que forman parte del grafo.
     */
    private List<NodeHex> nodes = new ArrayList<>();

    /**
     * Agrega un nodo al grafo.
     *
     * @param node El nodo que se desea agregar.
     */
    public void addNode(NodeHex node) {
        nodes.add(node);
    }

    /**
     * Obtiene la lista de nodos del grafo.
     *
     * @return Una lista con los nodos del grafo.
     */
    public List<NodeHex> getNodes() {
        return nodes;
    }

    /**
     * Establece la lista de nodos del grafo.
     *
     * @param nodes La nueva lista de nodos.
     */
    public void setNodes(List<NodeHex> nodes) {
        this.nodes = nodes;
    }
}
