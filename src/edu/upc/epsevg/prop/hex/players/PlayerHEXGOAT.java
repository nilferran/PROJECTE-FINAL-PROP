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
 * Clase que implementa un jugador automático para el juego Hex.
 * Este jugador utiliza el algoritmo Minimax con mejoras con Poda Alpha-Beta
 * 
 * @author JuanNil
 */

public class PlayerHEXGOAT implements IPlayer, IAuto {

    private String name;
    private int depthLimit;
    private boolean timeout;
    private int nodesExplorados;
    private Map<String, Integer> transpositionTable = new HashMap<>();
    private Map<String, Point> bestMoveTable = new HashMap<>();
    private GraphHex graph;
    private int turnos=0;
    private Utils utils = new Utils();

    
        /**
     * Constructor de la clase PlayerHEXGOAT.
     * 
     * @param name       Nombre del jugador.
     * @param depthLimit Límite de profundidad para la búsqueda.
     */
    
    public PlayerHEXGOAT(String name, int depthLimit ) {
        this.name = name;
        this.depthLimit = depthLimit;
        this.graph = new GraphHex();
    }
    
    /**
     * Indica que el tiempo límite ha sido alcanzado.
     */
    @Override
    public void timeout() {
        timeout = true;
    }

        /**
     * Devuelve el nombre del jugador.
     * 
     * @return Nombre del jugador.
     */
    
    @Override
    public String getName() {
        return "Minimax(" + name + ")";
    }

    
        /**
     * Realiza un movimiento en el juego utilizando el algoritmo Minimax con Poda Alpha-Beta.
     * 
     * @param s Estado actual del juego.
     * @return Movimiento calculado por el jugador.
     */
    
    @Override
    public PlayerMove move(HexGameStatus s) {
        turnos++;
        nodesExplorados=0;
        //System.out.println("Turno:" + turnos);

        Point bestMove = null;
        int bestValue = Integer.MIN_VALUE;
        int currentPlayer = mapPlayerTypeToInt(s.getCurrentPlayer());
        int profunditat = 1;
        PlayerMove mejorJugada = null;

        // Obtener los movimientos válidos (de MoveNode a Point)
        List<Point> validMoves = new ArrayList<>();
        Set<Point> moves;
        Set<Point> allStones = utils.getNonColorPoints(s, 0);

            if(allStones.isEmpty()) //Mapa vacio
            {
                //SI EMPIEZO Y NO HAY NINGUNA FICHA PUESTA, COMIENZO POR LA MITAD DEL TABLERO.
                HexGameStatus copy = new HexGameStatus(s);
                int _mitadTablero;
                _mitadTablero= s.getSize()/2;
                Point _firstMov = new Point (_mitadTablero,_mitadTablero);
                PlayerMove move = new PlayerMove(_firstMov, profunditat, 0, SearchType.MINIMAX);
               // System.out.println("Soy primero, Stone en el medio del tablero");
                return move;
            }
            //System.out.println(allStones.isEmpty());
            moves = new HashSet<>();
            allStones.stream()
            .map(point -> utils.getAllNeighborColor(s, point,0))
            .forEach(moves::addAll);
        HexGameStatus copy = new HexGameStatus(s);


        //REVISA SI HAY ALGUN MOVIMIENTO GANADOR, SI NO HAY, REALIZA EL ALGORITMO.
            int color = s.getCurrentPlayerColor();
            //System.out.println("color: " +color);
            Point p = utils.checkMoves(copy, moves, color);
            if (p != null)
            { 
                PlayerMove move = new PlayerMove(p, profunditat, 0, SearchType.MINIMAX);
                return move;
            }
            //System.out.println("Turno 2?" + utils.isTurn2(copy));
                    if (utils.isTurn2(s)) {
                  // System.out.println("En turno 2 ");
                // Obtener las piedras del rival
                Set<Point> rivalStones = utils.getColorPoints(s,1 );  // -currentPlayer es el rival
               // System.out.println(rivalStones);
                if (!rivalStones.isEmpty()) {
                    Point rivalFirstMove = rivalStones.iterator().next();  // Obtener la primera piedra del rival
                   // System.out.println("Primera piedra del rival en: " + rivalFirstMove);

                    // Buscar los vecinos vacíos de la piedra rival
                    List<Point> emptyNeighbors = utils.getEmptyNeighbor(s, rivalFirstMove);
                    if (!emptyNeighbors.isEmpty()) {
                        // Seleccionar el primer vecino vacío para colocar la ficha
                        bestMove = emptyNeighbors.get(0);
                        //System.out.println("Moviendo a: " + bestMove);

                        PlayerMove move = new PlayerMove(bestMove, nodesExplorados, 0, SearchType.MINIMAX);
                        return move;
                    }
                }
            }
               // System.out.println("No es turno 2 ");
                for (MoveNode moveNode : s.getMoves()) {
                    validMoves.add(moveNode.getPoint());
                }

                for (MoveNode moveNode : s.getMoves()) {

                    Point move = moveNode.getPoint();
                    if (s.getPos(move) == 0) { // Solo considerar celdas vacías
                        HexGameStatus nextState = new HexGameStatus(s);
                                  //  System.out.println("move turno antes????"+utils.isTurn2(nextState));
                        nextState.placeStone(move);
                        // Evaluar el movimiento usando minValue
                                   // System.out.println("move turno despues????"+utils.isTurn2(nextState));

                        int value = minValue(nextState, depthLimit - 1, Integer.MIN_VALUE, Integer.MAX_VALUE, currentPlayer);
                        //int value = minValue(nextState, depthLimit, Integer.MIN_VALUE, Integer.MAX_VALUE, currentPlayer);
                        // Actualizar el mejor movimiento
                        if (value > bestValue) {
                                bestValue = value;
                                bestMove = move;
                        }
                    }
                }

            mejorJugada = new PlayerMove(bestMove, nodesExplorados, depthLimit, SearchType.MINIMAX);
        return mejorJugada;
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
    if (depth == 0 || s.isGameOver()) {
        //System.out.println("turno: "+turnos + " y el current es: " + currentPlayer);

        return heuristica(s, currentPlayer);
        //return 0;

    } 

    int maxEval = Integer.MIN_VALUE;

    for (MoveNode moveNode : s.getMoves()) {
        Point move = moveNode.getPoint();
        if (s.getPos(move) == 0) {
            HexGameStatus nextState = new HexGameStatus(s);
            nextState.placeStone(move);

            int eval = minValue(nextState, depth - 1, alpha, beta, currentPlayer);
            maxEval = Math.max(maxEval, eval);
            alpha = Math.max(alpha, eval);

            if (beta <= alpha) break;
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
    if (depth == 0 || s.isGameOver()) {
        return heuristica(s, currentPlayer);
        //return 0;
    }

    int minEval = Integer.MAX_VALUE;

    for (MoveNode moveNode : s.getMoves()) {
        Point move = moveNode.getPoint();

        if (s.getPos(move) == 0) {
            //System.out.println("turno: "+turnos + " y el current es: " + currentPlayer);

            HexGameStatus nextState = new HexGameStatus(s);

            nextState.placeStone(move);

            int eval = maxValue(nextState, depth - 1, alpha, beta, currentPlayer);
            minEval = Math.min(minEval, eval);
            beta = Math.min(beta, eval);

            if (beta <= alpha) break;
        }
    }

    return minEval;
}

        /**
     * Calcula la heurística para evaluar el estado del juego.
     * 
     * @param s             Estado actual del juego.
     * @param currentPlayer Jugador actual.
     * @return Valor heurístico calculado.
     */
   private int heuristica(HexGameStatus s, int currentPlayer) {
    
    if(currentPlayer == 1){
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

         // Manejar escenarios en los que uno de los jugadores ya tiene un camino ganado
         if (playerScore == 0) return 999999;  // Ganó el jugador actual
         if (opponentScore == 0) return -999999; // Ganó el oponente

         // Retornar la diferencia como heurística
         return (int) (playerScore - opponentScore);
    }
    
    else if (currentPlayer == -1) {
        Utils utils = new Utils();
        int boardSize = s.getSize();

        // Identificar la posición inicial del rival
        Point rivalFirstMove = null;
        Set<Point> rivalStones = utils.getColorPoints(s, 1); // 1 representa al rival
        if (!rivalStones.isEmpty()) {
            rivalFirstMove = rivalStones.iterator().next(); // Primera piedra del rival
        }

        // Inicializar grafos para ambos jugadores
        GraphHex playerGraph = utils.initializeGraphHex(s, currentPlayer);
        GraphHex opponentGraph = utils.initializeGraphHex(s, -currentPlayer);

        // Calcular caminos más cortos
        utils.calculateShortestPath(playerGraph);
        utils.calculateShortestPath(opponentGraph);

        // Obtener puntuaciones iniciales
        float playerScore = utils.getScoreFromPath(playerGraph, s, currentPlayer);
        float opponentScore = utils.getScoreFromPath(opponentGraph, s, -currentPlayer);

        // Penalizar caminos del oponente
        float blockWeight = 2.0f; // Peso para bloquear al oponente
        if (rivalFirstMove != null) {
            List<Point> criticalNeighbors = utils.getEmptyNeighbor(s, rivalFirstMove);
            for (Point p : criticalNeighbors) {
                if (utils.isWithinBounds(p, boardSize) && s.getPos(p) == 0) {
                    opponentScore += blockWeight;
                }
            }
        }

        // Priorizar construcción de camino propio
        float ownPathWeight = 1.5f; // Peso para construir camino propio
        if (playerScore > 0) {
            playerScore -= ownPathWeight * playerScore / boardSize;
        }

        // Condiciones de victoria
        if (playerScore == 0) return 999999;  // Ganó el jugador actual
        if (opponentScore == 0) return -999999; // Ganó el oponente

        // Validación de puntuaciones y retorno final
        return (int) (playerScore - opponentScore);
        }

        return 0;

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