package edu.upc.epsevg.prop.hex.players;

import edu.upc.epsevg.prop.hex.HexGameStatus;
import edu.upc.epsevg.prop.hex.IAuto;
import edu.upc.epsevg.prop.hex.IPlayer;
import edu.upc.epsevg.prop.hex.PlayerMove;
import edu.upc.epsevg.prop.hex.SearchType;
import java.awt.Point;

/**
 * Jugador MiniMax amb poda alfa-beta
 */
public class PlayerMiniMax implements IPlayer, IAuto {

    private String name;
    private int depthLimit;

    public PlayerMiniMax(String name, int depthLimit) {
        this.name = name;
        this.depthLimit = depthLimit;
    }

    @Override
    public void timeout() {
        // No fem res específic per al timeout.
    }

    @Override
    public String getName() {
        return "MiniMax(" + name + ")";
    }

    /**
     * Decideix el moviment del jugador utilitzant MiniMax amb poda alfa-beta.
     *
     * @param s Tauler i estat actual de joc.
     * @return el moviment que fa el jugador.
     */
    @Override
    public PlayerMove move(HexGameStatus s) {
        long startTime = System.currentTimeMillis();

        Point bestMove = null;
        int bestValue = Integer.MIN_VALUE;

        // MiniMax amb poda alfa-beta
        for (int x = 0; x < s.getSize(); x++) {
            for (int y = 0; y < s.getSize(); y++) {
                if (s.getPos(new Point(x, y)) == 0) { // Casella buida
                    int[][] board = copyBoard(s);
                    board[x][y] = 1; // Aplica el moviment del jugador
                    int value = minimax(board, depthLimit - 1, false, Integer.MIN_VALUE, Integer.MAX_VALUE, s);
                    if (value > bestValue) {
                        bestValue = value;
                        bestMove = new Point(x, y);
                    }
                }
            }
        }

        long elapsedTime = System.currentTimeMillis() - startTime;
        return new PlayerMove(bestMove, elapsedTime, depthLimit, SearchType.MINIMAX);
    }

    /**
     * Implementació de MiniMax amb poda alfa-beta.
     *
     * @param board Estat actual del joc representat com una matriu.
     * @param depth Profunditat restant per explorar.
     * @param isMaximizing Indica si és el torn del jugador maximitzador.
     * @param alpha El valor alfa per a la poda.
     * @param beta El valor beta per a la poda.
     * @param s Tauler i estat actual de joc.
     * @return El valor heurístic del millor moviment.
     */
    private int minimax(int[][] board, int depth, boolean isMaximizing, int alpha, int beta, HexGameStatus s) {
        if (depth == 0 || isGameOver(board)) {
            return evaluate(board, s);
        }

        if (isMaximizing) {
            int maxEval = Integer.MIN_VALUE;
            for (int x = 0; x < board.length; x++) {
                for (int y = 0; y < board[x].length; y++) {
                    if (board[x][y] == 0) { // Casella buida
                        board[x][y] = 1; // Aplica el moviment del jugador
                        int eval = minimax(board, depth - 1, false, alpha, beta, s);
                        board[x][y] = 0; // Desfà el moviment del jugador
                        maxEval = Math.max(maxEval, eval);
                        alpha = Math.max(alpha, eval);
                        if (beta <= alpha) {
                            return maxEval; // Poda
                        }
                    }
                }
            }
            return maxEval;
        } else {
            int minEval = Integer.MAX_VALUE;
            for (int x = 0; x < board.length; x++) {
                for (int y = 0; y < board[x].length; y++) {
                    if (board[x][y] == 0) { // Casella buida
                        board[x][y] = 2; // Aplica el moviment de l'oponent
                        int eval = minimax(board, depth - 1, true, alpha, beta, s);
                        board[x][y] = 0; // Desfà el moviment de l'oponent
                        minEval = Math.min(minEval, eval);
                        beta = Math.min(beta, eval);
                        if (beta <= alpha) {
                            return minEval; // Poda
                        }
                    }
                }
            }
            return minEval;
        }
    }

    /**
     * Funció heurística per avaluar l'estat del joc.
     *
     * @param board Estat actual del joc representat com una matriu.
     * @param s Tauler i estat actual de joc.
     * @return El valor heurístic de l'estat.
     */
    private int evaluate(int[][] board, HexGameStatus s) {
        // Exemple senzill: prioritzar posicions centrals
        int center = board.length / 2;
        int score = 0;
        for (int x = 0; x < board.length; x++) {
            for (int y = 0; y < board[x].length; y++) {
                if (board[x][y] == 1) { // La nostra peça
                    score += center - Math.abs(center - x) + center - Math.abs(center - y);
                } else if (board[x][y] == 2) { // Peça de l'oponent
                    score -= center - Math.abs(center - x) + center - Math.abs(center - y);
                }
            }
        }
        return score;
    }

    /**
     * Crea una còpia de l'estat del tauler.
     *
     * @param s Estat actual del joc.
     * @return Una còpia de l'estat del tauler.
     */
    private int[][] copyBoard(HexGameStatus s) {
        int[][] newBoard = new int[s.getSize()][s.getSize()];
        for (int x = 0; x < s.getSize(); x++) {
            for (int y = 0; y < s.getSize(); y++) {
                newBoard[x][y] = s.getPos(new Point(x, y));
            }
        }
        return newBoard;
    }

    /**
     * Comprova si el joc ha acabat.
     *
     * @param board Estat actual del joc representat com una matriu.
     * @return Cert si el joc ha acabat, fals altrament.
     */
    private boolean isGameOver(int[][] board) {
    // Verifica si el jugador 1 ha ganado
    boolean[][] visited = new boolean[board.length][board.length];
    for (int x = 0; x < board.length; x++) {
        if (board[x][0] == 1 && hasPathToOppositeSide(board, visited, x, 0, 1)) {
            return true; // Jugador 1 ha ganado
        }
    }

    // Verifica si el jugador 2 ha ganado
    visited = new boolean[board.length][board.length];
    for (int y = 0; y < board.length; y++) {
        if (board[0][y] == 2 && hasPathToOppositeSide(board, visited, 0, y, 2)) {
            return true; // Jugador 2 ha ganado
        }
    }

    return false; // Ningún jugador ha ganado aún
}

    private boolean hasPathToOppositeSide(int[][] board, boolean[][] visited, int x, int y, int player) {
        if (player == 1 && y == board.length - 1) {
            return true; // Jugador 1 ha alcanzado el lado opuesto
        }
        if (player == 2 && x == board.length - 1) {
            return true; // Jugador 2 ha alcanzado el lado opuesto
        }

        visited[x][y] = true;

        int[] dx = {-1, -1, 0, 0, 1, 1};
        int[] dy = {-1, 0, -1, 1, 0, 1};

        for (int dir = 0; dir < 6; dir++) {
            int newX = x + dx[dir];
            int newY = y + dy[dir];

            if (isValidMove(board, newX, newY) && !visited[newX][newY] && board[newX][newY] == player) {
                if (hasPathToOppositeSide(board, visited, newX, newY, player)) {
                    return true;
                }
            }
        }

        return false;
    }

    private boolean isValidMove(int[][] board, int x, int y) {
        return x >= 0 && x < board.length && y >= 0 && y < board.length;
    }

    }
