package edu.upc.epsevg.prop.hex.players;

import edu.upc.epsevg.prop.hex.players.Dijkstra.GraphHex;
import edu.upc.epsevg.prop.hex.players.Dijkstra.NodeHex;
import edu.upc.epsevg.prop.hex.HexGameStatus;
import edu.upc.epsevg.prop.hex.IAuto;
import edu.upc.epsevg.prop.hex.IPlayer;
import edu.upc.epsevg.prop.hex.MoveNode;
import edu.upc.epsevg.prop.hex.PlayerMove;
import edu.upc.epsevg.prop.hex.PlayerType;
import edu.upc.epsevg.prop.hex.SearchType;
import java.awt.Point;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

/**
 * Clase que implementa un jugador automático para el juego Hex utilizando
 * el algoritmo Minimax con Deepening Iterativo (IDS).
 * Este jugador está optimizado para realizar búsquedas eficientes considerando
 * límites de tiempo y recursos.
 * 
 * @author JuanNil
 */

public class PlayerHEXGOATID implements IPlayer, IAuto {

    private String name;
    private int depthLimit;
    private boolean timeout=false;
    private int nodesExplorados;
    private Map<String, Integer> transpositionTable = new HashMap<>();
    private Map<String, Point> bestMoveTable = new HashMap<>();
    private GraphHex graph;
    private int turnos=0;
    private Utils utils = new Utils();

        /**
     * Constructor de la clase PlayerHEXGOATID.
     * 
     * @param name Nombre del jugador.
     */
    
    public PlayerHEXGOATID(String name) {
        this.name = name;
        this.depthLimit = depthLimit;
        this.graph = new GraphHex();
    }

        /**
     * Marca que se ha alcanzado el tiempo límite durante la búsqueda.
     */
    @Override
    public void timeout() {
        this.timeout = true;
    }

        /**
     * Devuelve el nombre del jugador.
     * 
     * @return Nombre del jugador.
     */
    @Override
    public String getName() {
        return "Minimax ID(" + name + ")";
    }

        /**
     * Realiza un movimiento calculado utilizando Minimax con Deepening Iterativo.
     * 
     * @param s Estado actual del juego.
     * @return Movimiento calculado por el jugador.
     */
    
@Override
    public PlayerMove move(HexGameStatus s) {
    turnos++;
    this.timeout = false;
    nodesExplorados=0;
    // Variables para controlar la búsqueda iterativa
    Point bestMove = null;
    int bestValue = Integer.MIN_VALUE;
    int currentPlayer = mapPlayerTypeToInt(s.getCurrentPlayer());
    int profunditat = 1; // Profundidad inicial
    PlayerMove mejorJugada = null;
    Set<Point> allStones = utils.getNonColorPoints(s, 0);

    
        // Si el tablero está vacío, colocar en el centro
    if (allStones.isEmpty()) {
        int mitadTablero = s.getSize() / 2;
        Point firstMove = new Point(mitadTablero, mitadTablero);
        return new PlayerMove(firstMove, profunditat, 0, SearchType.MINIMAX_IDS);
    }
    
    
    // Obtener los movimientos válidos (de MoveNode a Point)
    List<Point> validMoves = new ArrayList<>();
    for (MoveNode moveNode : s.getMoves()) {
        validMoves.add(moveNode.getPoint());
    }
    // Bucle de Iterative Deepening
    try {
        while (!timeout) {
            for (Point move : validMoves) {
                if (timeout) break; // Detener si se alcanza el timeout

                if (s.getPos(move) == 0) { // Considerar solo celdas vacías
                    HexGameStatus nextState = new HexGameStatus(s);
                    nextState.placeStone(move);

                    // Evaluar el movimiento usando minValue
                    int value = minValue(nextState, profunditat - 1, Integer.MIN_VALUE, Integer.MAX_VALUE, currentPlayer);

                    // Actualizar el mejor movimiento
                    if (value > bestValue) {
                        bestValue = value;
                        bestMove = move;
                    }
                }
            }

            if (timeout) break; // Verificar timeout después de cada profundidad

            // Actualizar la mejor jugada encontrada hasta esta profundidad
            if (bestMove != null) {
                mejorJugada = new PlayerMove(bestMove, nodesExplorados, profunditat, SearchType.MINIMAX_IDS);
            }

            // Incrementar la profundidad para la siguiente iteración
            profunditat++;
        }
    } 
    catch (Exception e) {
        System.err.println("Error durante la búsqueda: " + e.getMessage());
    }


    // Retornar el mejor movimiento encontrado hasta ahora
    return mejorJugada != null ? mejorJugada : new PlayerMove(validMoves.get(0), profunditat, 0, SearchType.MINIMAX_IDS);
}

        /**
     * Función Max del algoritmo Minimax con Poda Alpha-Beta.
     * 
     * @param s             Estado actual del juego.
     * @param depth         Profundidad restante.
     * @param alpha         Valor Alpha.
     * @param beta          Valor Beta.
     * @param currentPlayer Jugador actual.
     * @return Valor máximo calculado.
     */
private int maxValue(HexGameStatus s, int depth, int alpha, int beta, int currentPlayer) {
    nodesExplorados++;
    if (timeout) return Integer.MIN_VALUE; // Detener si se alcanza el timeout

    if (depth == 0 || s.isGameOver()) {
        return heuristica(s, currentPlayer);
    }

    int maxEval = Integer.MIN_VALUE;

    for (MoveNode moveNode : s.getMoves()) {
        if (timeout) break; // Detener si se alcanza el timeout

        Point move = moveNode.getPoint();
        if (s.getPos(move) == 0) {
            HexGameStatus nextState = new HexGameStatus(s);
            nextState.placeStone(move);

            int eval = minValue(nextState, depth - 1, alpha, beta, currentPlayer);
            maxEval = Math.max(maxEval, eval);
            alpha = Math.max(alpha, eval);

            if (beta <= alpha) break; // Poda alpha-beta
        }
    }

    return maxEval;
}


    /**
     * Función Min del algoritmo Minimax con Poda Alpha-Beta.
     * 
     * @param s             Estado actual del juego.
     * @param depth         Profundidad restante.
     * @param alpha         Valor Alpha.
     * @param beta          Valor Beta.
     * @param currentPlayer Jugador actual.
     * @return Valor mínimo calculado.
     */

private int minValue(HexGameStatus s, int depth, int alpha, int beta, int currentPlayer) {
    nodesExplorados++;

    if (timeout) return Integer.MAX_VALUE; // Detener si se alcanza el timeout

    if (depth == 0 || s.isGameOver()) {
        return heuristica(s, currentPlayer);
    }

    int minEval = Integer.MAX_VALUE;

    for (MoveNode moveNode : s.getMoves()) {
        if (timeout) break; // Detener si se alcanza el timeout

        Point move = moveNode.getPoint();
        if (s.getPos(move) == 0) {
            HexGameStatus nextState = new HexGameStatus(s);
            nextState.placeStone(move);

            int eval = maxValue(nextState, depth - 1, alpha, beta, currentPlayer);
            minEval = Math.min(minEval, eval);
            beta = Math.min(beta, eval);

            if (beta <= alpha) break; // Poda alpha-beta
        }
    }

    return minEval;
}


private List<Point> getPrioritizedMoves(HexGameStatus s, int depth) {
    List<Point> validMoves = new ArrayList<>();
    for (MoveNode moveNode : s.getMoves()) {
        validMoves.add(moveNode.getPoint());
    }

    // Generar la clave del estado usando `toString` y `depth`
    String stateKey = s.toString() + depth;

    // Priorizar el mejor movimiento conocido
    if (bestMoveTable.containsKey(stateKey)) {
        Point bestMove = bestMoveTable.get(stateKey);
        validMoves.remove(bestMove);
        validMoves.add(0, bestMove); // Mover al frente
    }

    return validMoves;
}


    /**
     * Calcula la heurística para evaluar el estado del juego.
     * 
     * @param s             Estado actual del juego.
     * @param currentPlayer Jugador actual.
     * @return Valor heurístico calculado.
     */

   private int heuristica(HexGameStatus s, int currentPlayer) {
    int boardSize = s.getSize();
    int boardCenter = boardSize / 2; // Coordenadas aproximadas del centro del tablero
   // System.out.println("hola "+ utils.isTurn2(s));

    // Inicializar los grafos para el jugador actual y el oponente
    GraphHex playerGraph = utils.initializeGraphHex(s, currentPlayer);
    GraphHex opponentGraph = utils.initializeGraphHex(s, -currentPlayer);

    // Calcular los caminos más cortos para ambos grafos
    utils.calculateShortestPath(playerGraph);
    utils.calculateShortestPath(opponentGraph);

    // Obtener las puntuaciones basadas en los caminos más cortos
    float playerScore = utils.getScoreFromPath(playerGraph, s, currentPlayer);
    float opponentScore = utils.getScoreFromPath(opponentGraph, s, -currentPlayer);
    //System.out.println("Player score"+ playerScore);
    //System.out.println("Rival score"+ opponentScore);

    // Manejar escenarios en los que uno de los jugadores ya tiene un camino ganado
    if (playerScore == 0) return 999999;  // Ganó el jugador actual
    if (opponentScore == 0)return -999999; // Ganó el oponente
    //if(playerScore == -999999) return -999999;
    //if(opponentScore == -999999) return 999999;
    // Retornar la diferencia como heurística
    return (int) (playerScore - opponentScore);
}


       /**
     * Convierte el tipo de jugador a un valor entero.
     * 
     * @param playerType Tipo de jugador (PLAYER1 o PLAYER2).
     * @return {@code 1} si es PLAYER1, {@code -1} si es PLAYER2.
     */

    private int mapPlayerTypeToInt(PlayerType playerType) {
            return playerType == PlayerType.PLAYER1 ? 1 : -1;
        }
    
}

