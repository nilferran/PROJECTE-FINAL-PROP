package edu.upc.epsevg.prop.hex.players;

import edu.upc.epsevg.prop.hex.HexGameStatus;
import edu.upc.epsevg.prop.hex.IAuto;
import edu.upc.epsevg.prop.hex.IPlayer;
import edu.upc.epsevg.prop.hex.PlayerMove;
import edu.upc.epsevg.prop.hex.SearchType;
import java.awt.Point;
import java.util.PriorityQueue;
import java.util.Comparator;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Jugador que utiliza Minimax con Dijkstra para decidir su movimiento
 */
public class MinimaxPlayer implements IPlayer, IAuto {

    private String name;
    private int maxDepth;
    private boolean useAlphaBeta;
    private long nodesExplored;
    private int maxDepthReached;

    public MinimaxPlayer(String name, int maxDepth, boolean useAlphaBeta) {
        this.name = name;
        this.maxDepth = maxDepth;
        this.useAlphaBeta = useAlphaBeta;
        this.nodesExplored = 0;
        this.maxDepthReached = 0;
    }

    @Override
    public void timeout() {
        // Nothing to do! I'm so fast, I never timeout 8-)
    }

    @Override
    public PlayerMove move(HexGameStatus s) {
        // Reinicia los contadores
        nodesExplored = 0;
        maxDepthReached = 0;

        Point bestMove = minimaxDecision(s, maxDepth, useAlphaBeta, new ArrayList<>(), 0);

        PlayerMove move = new PlayerMove(bestMove, 0L, 0, SearchType.MINIMAX);
        move.setNumerOfNodesExplored(nodesExplored);
        move.setMaxDepthReached(maxDepthReached);

        return move;
    }

    private Point minimaxDecision(HexGameStatus s, int depth, boolean useAlphaBeta, List<Point> moves, int currentDepth) {
        double bestValue = Double.NEGATIVE_INFINITY;
        Point bestMove = null;

        for (int i = 0; i < s.getSize(); i++) {
            for (int j = 0; j < s.getSize(); j++) {
                if (s.getPos(i, j) == 0 && !moves.contains(new Point(i, j))) {
                    nodesExplored++;
                    moves.add(new Point(i, j));
                    double value;
                    if (useAlphaBeta) {
                        value = minValueAB(s, depth - 1, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, moves, currentDepth + 1);
                    } else {
                        value = minValue(s, depth - 1, moves, currentDepth + 1);
                    }
                    moves.remove(moves.size() - 1);
                    if (value > bestValue) {
                        bestValue = value;
                        bestMove = new Point(i, j);
                    }
                }
            }
        }

        return bestMove;
    }

    private double maxValue(HexGameStatus s, int depth, List<Point> moves, int currentDepth) {
        maxDepthReached = Math.max(maxDepthReached, currentDepth);
        if (depth == 0 || s.isGameOver()) {
            return evaluate(s, moves);
        }
        double value = Double.NEGATIVE_INFINITY;
        for (int i = 0; i < s.getSize(); i++) {
            for (int j = 0; j < s.getSize(); j++) {
                if (s.getPos(i, j) == 0 && !moves.contains(new Point(i, j))) {
                    nodesExplored++;
                    moves.add(new Point(i, j));
                    value = Math.max(value, minValue(s, depth - 1, moves, currentDepth + 1));
                    moves.remove(moves.size() - 1);
                }
            }
        }
        return value;
    }

    private double minValue(HexGameStatus s, int depth, List<Point> moves, int currentDepth) {
        maxDepthReached = Math.max(maxDepthReached, currentDepth);
        if (depth == 0 || s.isGameOver()) {
            return evaluate(s, moves);
        }
        double value = Double.POSITIVE_INFINITY;
        for (int i = 0; i < s.getSize(); i++) {
            for (int j = 0; j < s.getSize(); j++) {
                if (s.getPos(i, j) == 0 && !moves.contains(new Point(i, j))) {
                    nodesExplored++;
                    moves.add(new Point(i, j));
                    value = Math.min(value, maxValue(s, depth - 1, moves, currentDepth + 1));
                    moves.remove(moves.size() - 1);
                }
            }
        }
        return value;
    }

    private double maxValueAB(HexGameStatus s, int depth, double alpha, double beta, List<Point> moves, int currentDepth) {
        maxDepthReached = Math.max(maxDepthReached, currentDepth);
        if (depth == 0 || s.isGameOver()) {
            return evaluate(s, moves);
        }
        double value = Double.NEGATIVE_INFINITY;
        for (int i = 0; i < s.getSize(); i++) {
            for (int j = 0; j < s.getSize(); j++) {
                if (s.getPos(i, j) == 0 && !moves.contains(new Point(i, j))) {
                    nodesExplored++;
                    moves.add(new Point(i, j));
                    value = Math.max(value, minValueAB(s, depth - 1, alpha, beta, moves, currentDepth + 1));
                    moves.remove(moves.size() - 1);
                    if (value >= beta) return value;
                    alpha = Math.max(alpha, value);
                }
            }
        }
        return value;
    }

    private double minValueAB(HexGameStatus s, int depth, double alpha, double beta, List<Point> moves, int currentDepth) {
        maxDepthReached = Math.max(maxDepthReached, currentDepth);
        if (depth == 0 || s.isGameOver()) {
            return evaluate(s, moves);
        }
        double value = Double.POSITIVE_INFINITY;
        for (int i = 0; i < s.getSize(); i++) {
            for (int j = 0; j < s.getSize(); j++) {
                if (s.getPos(i, j) == 0 && !moves.contains(new Point(i, j))) {
                    nodesExplored++;
                    moves.add(new Point(i, j));
                    value = Math.min(value, maxValueAB(s, depth - 1, alpha, beta, moves, currentDepth + 1));
                    moves.remove(moves.size() - 1);
                    if (value <= alpha) return value;
                    beta = Math.min(beta, value);
                }
            }
        }
        return value;
    }

    private double evaluate(HexGameStatus s, List<Point> moves) {
        int size = s.getSize();
        double[][] weights = new double[size][size];
        for (double[] row : weights) {
            Arrays.fill(row, Double.POSITIVE_INFINITY);
        }

        PriorityQueue<Node> pq = new PriorityQueue<>(Comparator.comparingDouble(node -> node.distance));
        for (int i = 0; i < size; i++) {
            pq.add(new Node(i, 0, 0));
            weights[i][0] = 0;
        }

        int[][] directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}, {-1, 1}, {1, -1}};
        while (!pq.isEmpty()) {
            Node current = pq.poll();
            for (int[] dir : directions) {
                int nx = current.x + dir[0];
                int ny = current.y + dir[1];
                if (isValid(nx, ny, size) && weights[nx][ny] > current.distance + 1) {
                    weights[nx][ny] = current.distance + 1;
                    pq.add(new Node(nx, ny, weights[nx][ny]));
                }
            }
        }

        double totalWeight = 0;
        for (Point move : moves) {
            int x = move.x;
            int y = move.y;
            totalWeight += weights[x][y];
        }

        return totalWeight;
    }

    private boolean isValid(int x, int y, int size) {
        return x >= 0 && x < size && y >= 0 && y < size;
    }

    @Override
    public String getName() {
        return "Jugador Minimax(" + name + ")";
    }

    private static class Node {
        int x, y;
        double distance;

        Node(int x, int y, double distance) {
            this.x = x;
            this.y = y;
            this.distance = distance;
        }
    }
}
