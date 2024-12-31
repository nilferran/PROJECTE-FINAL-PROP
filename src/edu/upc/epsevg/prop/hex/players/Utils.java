package edu.upc.epsevg.prop.hex.players;

import edu.upc.epsevg.prop.hex.players.Dijkstra. *;

import edu.upc.epsevg.prop.hex.HexGameStatus;

import java.awt.*;
import java.util.*;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;


/**
 * Clase Utils, funciones para nuestros jugadores Minimax. 
 * Contiene métodos para manejar nodos, calcular rutas más cortas y evaluar
 * diferentes aspectos del estado del juego.
 * @author JuanNil
 */


class Utils {
    /**
     * Direcciones vecinas para el cálculo de conexiones directas.
     */
    private List<Point> neighbor_directions = Arrays.asList(
            new Point(1, 0), new Point(1, -1), new Point(0, -1),
            new Point(-1, 0), new Point(-1, 1), new Point(0, 1));
     /**
     * Direcciones vecinas diagonales para conexiones avanzadas.
     */
    private List<Point> disgonal_neighbor_directions = Arrays.asList(
            new Point(2, -1), new Point(1, 1), new Point(-1, 2),
            new Point(-2, 1), new Point(-1, -1), new Point(1, -2));
    /**
     * Direcciones completas que combinan vecinos directos y diagonales.
     */
    private List<Point> all_neighbor_directions = Arrays.asList(
            new Point(1, 0), new Point(1, -1), new Point(0, -1),
            new Point(-1, 0), new Point(-1, 1), new Point(0, 1),
            new Point(2, -1), new Point(1, 1), new Point(-1, 2),
            new Point(-2, 1), new Point(-1, -1), new Point(1, -2));

    /**
     * Verifica si un conjunto de movimientos contiene movimientos ganadores.
     *
     * @param s Estado actual del juego.
     * @param l      Conjunto de movimientos a verificar.
     * @param color  Color del jugador actual.
     * @return El primer movimiento ganador encontrado o null si no existe.
     */
    
    Point checkMoves(HexGameStatus s, Set<Point> l, int color){
        List<Point> winL =  l.stream().parallel().filter(point -> {
            HexGameStatus nouTauler = new HexGameStatus(s);
            nouTauler.placeStone(point);
            return nouTauler.isGameOver();
        }).collect(Collectors.toList());

        return winL.isEmpty() ? null : winL.get(0);
    }

        /**
     * Obtiene puntos no ocupados por un color específico.
     *
     * @param s     Estado actual del juego.
     * @param color Color a excluir.
     * @return Conjunto de puntos no ocupados por el color especificado.
     */
    
    Set<Point> getNonColorPoints(HexGameStatus s, int color) {
        Set<Point> moves = new HashSet<>();
        Stream.iterate(0, n -> n + 1).limit(s.getSize())
                .forEach(i ->
                        moves.addAll(Stream.iterate(0, t -> t + 1).limit(s.getSize()).parallel()
                                .filter(j -> s.getPos(i, j) != color)
                                .map(j -> new Point(i, j))
                                .collect(Collectors.toList())));
        return moves;
    }
        /**
     * Obtiene puntos ocupados por un color específico.
     *
     * @param s     Estado actual del juego.
     * @param color Color a buscar.
     * @return Conjunto de puntos ocupados por el color especificado.
     */
    
    Set<Point> getColorPoints(HexGameStatus s, int color) {
        Set<Point> moves = new HashSet<>();
        Stream.iterate(0, n -> n + 1).limit(s.getSize())
                .forEach(i ->
                        moves.addAll(Stream.iterate(0, t -> t + 1).limit(s.getSize()).parallel()
                                .filter(j -> s.getPos(i, j) == color)
                                .map(j -> new Point(i, j))
                                .collect(Collectors.toList())));
        return moves;
    }

     /**
     * Suma dos puntos para calcular una nueva posición.
     *
     * @param p   Punto inicial.
     * @param dir Dirección a sumar.
     * @return Nuevo punto resultante.
     */
    
    private Point sumPoint(Point p, Point dir) {
        return new Point(p.x+dir.x, p.y+dir.y);
    }

        /**
     * Obtiene los vecinos de un punto que tienen un color específico.
     *
     * @param s     Estado actual del juego.
     * @param point Punto de referencia.
     * @param color Color a buscar.
     * @return Lista de puntos vecinos con el color especificado.
     */
    
    List<Point> getAllNeighborColor(HexGameStatus s, Point point, int color) {
    int boardSize = s.getSize();
    return all_neighbor_directions.stream()
            .map(direction -> sumPoint(point, direction))
            .filter(p -> isWithinBounds(p, boardSize)) // Validar límites
            .filter(p -> s.getPos(p.x, p.y) == color) // Verificar el color
            .collect(Collectors.toList());
}

    /**
     * Obtiene los vecinos vacíos de un punto.
     *
     * @param s     Estado actual del juego.
     * @param point Punto de referencia.
     * @return Lista de puntos vecinos vacíos.
     */
    
    List<Point> getEmptyNeighbor(HexGameStatus s, Point point) {
    int boardSize = s.getSize(); // Tamaño dinámico del tablero
    return neighbor_directions.stream()
            .map(direction -> sumPoint(point, direction))
            .filter(p -> isWithinBounds(p, boardSize)) // Validar límites con un método auxiliar
            .filter(p -> s.getPos(p.x, p.y) == 0) // Verificar si está vacía
            .collect(Collectors.toList());
}

    /**
     * Verifica si un punto está dentro de los límites del tablero.
     *
     * @param p         Punto a verificar.
     * @param boardSize Tamaño del tablero.
     * @return {@code true} si el punto está dentro de los límites, de lo contrario {@code false}.
     */
    
    public boolean isWithinBounds(Point p, int boardSize) {
    return p.x >= 0 && p.y >= 0 && p.x < boardSize && p.y < boardSize;
}


    /**
     * Direcciones vecinas hacia abajo para conexiones específicas.
     */
    private List<Point> down_neighbor_directions = Arrays.asList(
            new Point(1, 0), new Point(0, 1), new Point(-1, 1), new Point(-1, 0));
    /**
    * Direcciones vecinas hacia la derecha para conexiones específicas.
    */
    private List<Point> rigth_neighbor_directions = Arrays.asList(
            new Point(0, -1), new Point(1, -1), new Point(1, 0), new Point(0, 1));

     /**
     * Inicializa un grafo para representar el estado del juego basado en el color.
     *
     * @param s     Estado actual del juego.
     * @param color Color del jugador actual.
     * @return Un grafo con nodos y conexiones inicializados.
     */
    GraphHex initializeGraphHex(HexGameStatus s, int color){
        GraphHex graph = new GraphHex();
        List<NodeHex> nodes = getNonColorPoints(s,-color).stream().map(NodeHex::new).collect(Collectors.toList());

        if(color == 1){
            NodeHex end = new NodeHex(new Point(s.getSize(), 0));
            nodes.stream().parallel().filter(n -> n.getPoint().x == s.getSize()-1)
                    .forEach(n -> n.addDestination(end, 0));
            graph.addNode(end);

            setNodeConnections(s, color, graph, nodes, rigth_neighbor_directions, rigth_close_directions,
                    rigth_close_directions2, next_rigth_direction);

            NodeHex start = new NodeHex(new Point(-1,0));
            nodes.stream().parallel().filter(n -> n.getPoint().x == 0)
                    .forEach(n -> {
                            if (s.getPos(n.getPoint().x, n.getPoint().y) == color) start.addDestination(n, 0);
                            else start.addDestination(n, 4);
                    });
            graph.addNode(start);

        }else if(color == -1){
            NodeHex end = new NodeHex(new Point(0,s.getSize()));
            nodes.stream().parallel().filter(n -> n.getPoint().y == s.getSize()-1)
                    .forEach(n ->  n.addDestination(end, 0));
            graph.addNode(end);

            setNodeConnections(s, color, graph, nodes, down_neighbor_directions, down_close_direction,
                    down_close_direction2, next_down_direction);

            NodeHex start = new NodeHex(new Point(0,-1));
            nodes.stream().parallel().filter(n -> n.getPoint().y == 0)
                    .forEach(n -> {
                            if (s.getPos(n.getPoint().x, n.getPoint().y) == color) start.addDestination(n, 0);
                            else start.addDestination(n, 4);
                    });
            graph.addNode(start);
        }

        return graph;
    }
/**
 * Direcciones cerradas hacia abajo para evaluar conexiones bloqueadas.
 */
    private List<Point> down_close_direction = Arrays.asList(new Point(-1,0), new Point(0, 1));
    /**
 * Direcciones cerradas adicionales hacia abajo para conexiones extendidas.
 */
    private List<Point> down_close_direction2 = Arrays.asList(new Point(1,0), new Point(-1, 1));

    /**
 * Direcciones cerradas hacia la derecha para evaluar conexiones bloqueadas.
 */
    private List<Point>  rigth_close_directions = Arrays.asList(new Point(0,-1), new Point(1, 0));
    private List<Point>  rigth_close_directions2 = Arrays.asList(new Point(0,1), new Point(1, -1));

    /**
 * Direcciones para calcular movimientos hacia la derecha extendidos.
 */
    private List<Point>  next_rigth_direction = Arrays.asList(new Point(1,1), new Point(2, -1),new Point(1,-2));
    
   /**
 * Direcciones para calcular movimientos hacia abajo extendidos.
 */ 
    private List<Point>  next_down_direction = Arrays.asList(new Point(-1,2), new Point(1, 1), new Point(-2,1));

    
    
    /**
 * Establece las conexiones entre nodos en un grafo con base en las direcciones y posiciones.
 *
 * @param s                  Estado actual del juego.
 * @param color              Color del jugador actual.
 * @param graph              Grafo a modificar.
 * @param nodes              Lista de nodos en el grafo.
 * @param neighbor_directions Direcciones vecinas para considerar conexiones.
 * @param close_directions   Direcciones cerradas a considerar.
 * @param close_directions2  Direcciones cerradas adicionales.
 * @param next_direction     Direcciones extendidas para evaluar conexiones avanzadas.
 */
    
    private void setNodeConnections(HexGameStatus s, int color, GraphHex graph, List<NodeHex> nodes, List<Point> neighbor_directions,
                                    List<Point> close_directions, List<Point> close_directions2,
                                    List<Point> next_direction) {
        nodes.forEach(node -> {
            neighbor_directions.stream()
                    .map(direction -> sumPoint(node.getPoint(),direction))
                    .filter(p -> p.x>=0 && p.y>=0 && p.x <s.getSize() && p.y<s.getSize() && s.getPos(p.x, p.y) != -color)
                    .forEach(point -> {
                        int distance;
                        if (s.getPos(point.x, point.y) == 0){
                            if (isClossed(s, color, close_directions, close_directions2.get(1), point)
                                    || isClossed(s, color, close_directions2, close_directions.get(1), point)){
                                distance = 10;
                            }else {
                                distance = isDiagonal(s, color, neighbor_directions, next_direction, node, point) ? 1:6;
                            }
                        }else{
                            distance = 0;
                        }

                        int pos = nodes.indexOf(new NodeHex(point));
                        node.addDestination(nodes.get(pos), distance);
                    });
            graph.addNode(node);
        });
    }
/**
 * Verifica si un nodo y un punto están conectados de forma diagonal con respecto al color dado.
 *
 * @param s                  Estado actual del juego.
 * @param color              Color del jugador actual.
 * @param neighbor_directions Direcciones vecinas a considerar.
 * @param next_direction     Direcciones extendidas para evaluar conexiones diagonales.
 * @param node               Nodo de referencia.
 * @param point              Punto de referencia.
 * @return {@code true} si existe una conexión diagonal válida, de lo contrario {@code false}.
 */
    private boolean isDiagonal(HexGameStatus s, int color, List<Point> neighbor_directions, List<Point> next_direction,
                               NodeHex node, Point point){
        List<Point> dig = next_direction.stream().parallel()
                .map(direction -> sumPoint(node.getPoint(),direction))
                .filter(p -> p.x>=0 && p.y>=0 && p.x <s.getSize() && p.y<s.getSize() && s.getPos(p.x, p.y) == color)
                .collect(Collectors.toList());

        return neighbor_directions.stream().parallel()
                .map(direction -> sumPoint(point, direction))
                .anyMatch(p -> p.x >= 0 && p.y >= 0 && p.x < s.getSize() && p.y < s.getSize()
                        && s.getPos(p.x, p.y) == color && dig.contains(p));
    }

    /**
 * Verifica si un punto está bloqueado por piezas del oponente en direcciones específicas.
 *
 * @param s                Estado actual del juego.
 * @param color            Color del jugador actual.
 * @param close_directions Direcciones vecinas a considerar como bloqueadas.
 * @param dirNext          Dirección extendida a evaluar.
 * @param point            Punto de referencia.
 * @return {@code true} si el punto está bloqueado, de lo contrario {@code false}.
 */
    private boolean isClossed(HexGameStatus s, int color, List<Point> close_directions, Point dirNext, Point point) {
        boolean isColor = true;

        Point next = sumPoint(dirNext, point);
        if(next.x>=0 && next.y>=0 && next.x <s.getSize() && next.y<s.getSize()){
            isColor = s.getPos(next.x, next.y) != color;
        }
        return  isColor &&  close_directions.stream().parallel()
                    .map(direction -> sumPoint(point,direction))
                    .filter(p -> p.x>=0 && p.y>=0 && p.x <s.getSize() && p.y<s.getSize() && s.getPos(p.x, p.y) == -color)
                    .count() == 2;
    }

    /**
 * Calcula los caminos más cortos en un grafo utilizando el algoritmo de Dijkstra.
 *
 * @param grap Grafo en el cual se calcularán los caminos más cortos.
 */
    public void calculateShortestPath(GraphHex grap) {
        try{
            NodeHex source = grap.getNodes().get(grap.getNodes().size()-1);
            source.setDistance(0);

            Set<NodeHex> settledNodes = new HashSet<>();
            Set<NodeHex> unsettledNodes = new HashSet<>();

            unsettledNodes.add(source);
            while (unsettledNodes.size() != 0) {
                NodeHex currentNode = getLowestDistanceNode(unsettledNodes);
                unsettledNodes.remove(currentNode);
                currentNode.getAdjacentNodeHexs().forEach((adjacentNode, edgeWeight) -> {
                    if (!settledNodes.contains(adjacentNode)) {
                        CalculateMinimumDistance(adjacentNode, edgeWeight, currentNode);
                        unsettledNodes.add(adjacentNode);
                    }
                });
                settledNodes.add(currentNode);
            }
        }catch (Exception e){
            e.printStackTrace();
        }
    }

    /**
 * Obtiene el nodo con la distancia más baja entre los nodos no resueltos.
 *
 * @param unsettledNodes Conjunto de nodos no resueltos.
 * @return Nodo con la distancia más baja.
 */
    private NodeHex getLowestDistanceNode(Set <NodeHex> unsettledNodes) {
        NodeHex lowestDistanceNode = null;
        int lowestDistance = Integer.MAX_VALUE;
        for (NodeHex node: unsettledNodes) {
            int nodeDistance = node.getDistance();
            if (nodeDistance < lowestDistance) {
                lowestDistance = nodeDistance;
                lowestDistanceNode = node;
            }
        }
        return lowestDistanceNode;
    }

    /**
 * Actualiza la distancia mínima para un nodo en base a su nodo de origen y el peso de la arista.
 *
 * @param evaluationNode Nodo a evaluar.
 * @param edgeWeigh      Peso de la arista.
 * @param sourceNode     Nodo de origen.
 */
    private void CalculateMinimumDistance(NodeHex evaluationNode, Integer edgeWeigh, NodeHex sourceNode) {
        Integer sourceDistance = sourceNode.getDistance();
        if (sourceDistance + edgeWeigh < evaluationNode.getDistance()) {
            evaluationNode.setDistance(sourceDistance + edgeWeigh);
            LinkedList<NodeHex> shortestPath = new LinkedList<>(sourceNode.getShortestPath());
            shortestPath.add(sourceNode);
            evaluationNode.setShortestPath(shortestPath);
        }
    }


/**
 * Calcula una puntuación basada en las distancias del camino más corto en un grafo.
 *
 * @param g     Grafo a evaluar.
 * @param s     Estado actual del juego.
 * @param color Color del jugador actual.
 * @return Puntuación basada en las distancias.
 */
    float getScoreFromPath(GraphHex g, HexGameStatus s, int color){
        float distance;
        List<NodeHex> aux;
        int size= g.getNodes().get(0).getShortestPath().size();
        if(size == 0) return -999999;

        if(color == 1){
            aux = g.getNodes().stream()
                    .filter(n -> n.getPoint().x == s.getSize()-1).collect(Collectors.toList());
        }else {
            aux = g.getNodes().stream()
                    .filter(n -> n.getPoint().y == s.getSize()-1).collect(Collectors.toList());
        }

        distance = aux.stream().map(NodeHex::getDistance).reduce(0, Integer::sum);
        if(aux.size() < s.getSize()){ distance += s.getSize()*(s.getSize() - aux.size()); }

        float tt = g.getNodes().get(0).getShortestPath().get(size-1).getDistance();
        distance = tt + distance/6;

        return -distance;
    }
    
         /**
     * Inicializa un grafo para representar el estado del juego basado en el color.
     *
     * @param s     Estado actual del juego.
     * @return Si solo hay una ficha en el tablero, true.
     */
        // Verifica si estamos en el turno 2
    static boolean isTurn2(HexGameStatus s) {
        int occupiedCells = 0;
        int _size= s.getSize();
        for (int x = 0; x < _size; x++) {
            for (int y = 0; y < _size; y++) {
                Point board = new Point(x,y);
                if (s.getPos(board)!= 0) {
                    occupiedCells++;
                }
            }
        }
        return occupiedCells == 1;
    }

}