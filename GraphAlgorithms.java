import java.util.Set;
import java.util.List;
import java.util.Map;
import java.util.HashSet;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;

/**
 * Your implementation of various different graph algorithms.
 *
 * @author Quang Nguyen
 * @version 1.0
 * @userid qnguyen305
 * @GTID 903770019
 *
 * Collaborators: LIST ALL COLLABORATORS YOU WORKED WITH HERE
 *
 * Resources: LIST ALL NON-COURSE RESOURCES YOU CONSULTED HERE
 */
public class GraphAlgorithms {

    /**
     * Performs a breadth first search (bfs) on the input graph, starting at
     * the parameterized starting vertex.
     *
     * When exploring a vertex, explore in the order of neighbors returned by
     * the adjacency list. Failure to do so may cause you to lose points.
     *
     * You may import/use java.util.Set, java.util.List, java.util.Queue, and
     * any classes that implement the aforementioned interfaces, as long as they
     * are efficient.
     *
     * The only instance of java.util.Map that you may use is the
     * adjacency list from graph. DO NOT create new instances of Map
     * for BFS (storing the adjacency list in a variable is fine).
     *
     * DO NOT modify the structure of the graph. The graph should be unmodified
     * after this method terminates.
     *
     * @param <T>   the generic typing of the data
     * @param start the vertex to begin the bfs on
     * @param graph the graph to search through
     * @return list of vertices in visited order
     * @throws IllegalArgumentException if any input is null, or if start
     *                                  doesn't exist in the graph
     */
    public static <T> List<Vertex<T>> bfs(Vertex<T> start, Graph<T> graph) {
        if (start == null) {
            throw new IllegalArgumentException("start is null");
        }
        if (graph == null) {
            throw new IllegalArgumentException("graph is null");
        }

        Map<Vertex<T>, List<VertexDistance<T>>> adjList = graph.getAdjList();

        if (!adjList.containsKey(start)) {
            throw new IllegalArgumentException("start doesn't exist in the graph");
        }

        List<VertexDistance<T>> neighbors = adjList.get(start);

        HashSet<Vertex<T>> vs = new HashSet<>();
        Queue<Vertex<T>> queue = new LinkedList<>();
        ArrayList<Vertex<T>> ls = new ArrayList<>();

        queue.add(start);
        vs.add(start);

        while (!queue.isEmpty() && ls.size() < graph.getVertices().size()) {
            Vertex<T> v = queue.remove();
            ls.add(v);
            neighbors = adjList.get(v);
            for (VertexDistance<T> w : neighbors) {
                if (!vs.contains(w.getVertex())) {
                    queue.add(w.getVertex());
                    vs.add(w.getVertex());
                }
            }
        }
        return ls;
    }

    /**
     * Performs a depth first search (dfs) on the input graph, starting at
     * the parameterized starting vertex.
     *
     * When exploring a vertex, explore in the order of neighbors returned by
     * the adjacency list. Failure to do so may cause you to lose points.
     *
     * *NOTE* You MUST implement this method recursively, or else you will lose
     * all points for this method.
     *
     * You may import/use java.util.Set, java.util.List, and
     * any classes that implement the aforementioned interfaces, as long as they
     * are efficient.
     *
     * The only instance of java.util.Map that you may use is the
     * adjacency list from graph. DO NOT create new instances of Map
     * for DFS (storing the adjacency list in a variable is fine).
     *
     * DO NOT modify the structure of the graph. The graph should be unmodified
     * after this method terminates.
     *
     * @param <T>   the generic typing of the data
     * @param start the vertex to begin the dfs on
     * @param graph the graph to search through
     * @return list of vertices in visited order
     * @throws IllegalArgumentException if any input is null, or if start
     *                                  doesn't exist in the graph
     */
    public static <T> List<Vertex<T>> dfs(Vertex<T> start, Graph<T> graph) {
        if (start == null) {
            throw new IllegalArgumentException("start is null");
        }
        if (graph == null) {
            throw new IllegalArgumentException("graph is null");
        }
        Map<Vertex<T>, List<VertexDistance<T>>> adjList = graph.getAdjList();

        if (!adjList.containsKey(start)) {
            throw new IllegalArgumentException("start doesn't exist in the graph");
        }

        Set<Vertex<T>> vs = new HashSet<>();
        ArrayList<Vertex<T>> ls = new ArrayList<>();

        dfsRecursive(start, vs, ls, adjList);
        return ls;
    }

    /**
     *
     * @param start start
     * @param vs visited set
     * @param ls list
     * @param adjList adj list
     * @param <T> type
     */
    private static <T> void dfsRecursive(Vertex<T> start, Set<Vertex<T>> vs, ArrayList<Vertex<T>> ls,
                                                    Map<Vertex<T>, List<VertexDistance<T>>> adjList) {
        vs.add(start);
        ls.add(start);
        for (VertexDistance<T> v : adjList.get(start)) {
            if (!vs.contains(v.getVertex())) {
                dfsRecursive(v.getVertex(), vs, ls, adjList);
            }
        }
    }

    /**
     * Finds the single-source shortest distance between the start vertex and
     * all vertices given a weighted graph (you may assume non-negative edge
     * weights).
     *
     * Return a map of the shortest distances such that the key of each entry
     * is a node in the graph and the value for the key is the shortest distance
     * to that node from start, or Integer.MAX_VALUE (representing
     * infinity) if no path exists.
     *
     * You may import/use java.util.PriorityQueue,
     * java.util.Map, and java.util.Set and any class that
     * implements the aforementioned interfaces, as long as your use of it
     * is efficient as possible.
     *
     * You should implement the version of Dijkstra's where you use two
     * termination conditions in conjunction.
     *
     * 1) Check if all of the vertices have been visited.
     * 2) Check if the PQ is empty yet.
     *
     * DO NOT modify the structure of the graph. The graph should be unmodified
     * after this method terminates.
     *
     * @param <T>   the generic typing of the data
     * @param start the vertex to begin the Dijkstra's on (source)
     * @param graph the graph we are applying Dijkstra's to
     * @return a map of the shortest distances from start to every
     * other node in the graph
     * @throws IllegalArgumentException if any input is null, or if start
     *                                  doesn't exist in the graph.
     */
    public static <T> Map<Vertex<T>, Integer> dijkstras(Vertex<T> start,
                                                        Graph<T> graph) {
        if (start == null) {
            throw new IllegalArgumentException("start is null");
        }
        if (graph == null) {
            throw new IllegalArgumentException("graph is null");
        }
        Map<Vertex<T>, List<VertexDistance<T>>> adjList = graph.getAdjList();

        if (!adjList.containsKey(start)) {
            throw new IllegalArgumentException("start doesn't exist in the graph");
        }

        List<VertexDistance<T>> neighbors = adjList.get(start);

        HashSet<Vertex<T>> vs = new HashSet<>();
        PriorityQueue<VertexDistance<T>> queue = new PriorityQueue<>();
        HashMap<Vertex<T>, Integer> dm = new HashMap<>();

        for (Vertex<T> v : graph.getVertices()) {
            dm.put(v, Integer.MAX_VALUE);
        }

        queue.add(new VertexDistance<>(start, 0));
        dm.put(start, 0);

        while (!queue.isEmpty() && vs.size() < graph.getVertices().size()) {
            VertexDistance<T>  u = queue.remove();
            if (!vs.contains(u.getVertex())) {
                vs.add(u.getVertex());
                if (vs.size() >= graph.getVertices().size()) {
                    break;
                }
                neighbors = adjList.get(u.getVertex());
                for (VertexDistance<T> w : neighbors) {
                    if (!vs.contains(w.getVertex())) {
                        dm.put(w.getVertex(), Math.min(dm.get(w.getVertex()), u.getDistance() + w.getDistance()));
                        queue.add(new VertexDistance<>(w.getVertex(), u.getDistance() + w.getDistance()));
                    }
                }
            }
        }

        return dm;
    }

    /**
     * Runs Kruskal's algorithm on the given graph and returns the Minimal
     * Spanning Tree (MST) in the form of a set of Edges. If the graph is
     * disconnected and therefore no valid MST exists, return null.
     *
     * You may assume that the passed in graph is undirected. In this framework,
     * this means that if (u, v, 3) is in the graph, then the opposite edge
     * (v, u, 3) will also be in the graph, though as a separate Edge object.
     *
     * The returned set of edges should form an undirected graph. This means
     * that every time you add an edge to your return set, you should add the
     * reverse edge to the set as well. This is for testing purposes. This
     * reverse edge does not need to be the one from the graph itself; you can
     * just make a new edge object representing the reverse edge.
     *
     * You may assume that there will only be one valid MST that can be formed.
     *
     * Kruskal's will also require you to use a Disjoint Set which has been
     * provided for you. A Disjoint Set will keep track of which vertices are
     * connected given the edges in your current MST, allowing you to easily
     * figure out whether adding an edge will create a cycle. Refer
     * to the DisjointSet and DisjointSetNode classes that
     * have been provided to you for more information.
     *
     * An MST should NOT have self-loops or parallel edges.
     *
     * By using the Disjoint Set provided, you can avoid adding self-loops and
     * parallel edges into the MST.
     *
     * You may import/use java.util.PriorityQueue,
     * java.util.Set, and any class that implements the aforementioned
     * interfaces.
     *
     * DO NOT modify the structure of the graph. The graph should be unmodified
     * after this method terminates.
     *
     * @param <T>   the generic typing of the data
     * @param graph the graph we are applying Kruskals to
     * @return the MST of the graph or null if there is no valid MST
     * @throws IllegalArgumentException if any input is null
     */
    public static <T> Set<Edge<T>> kruskals(Graph<T> graph) {
        if (graph == null) {
            throw new IllegalArgumentException("graph is null");
        }

        DisjointSet<T> ds = new DisjointSet<>();
        Set<Edge<T>> s = new HashSet<>();
        PriorityQueue<Edge<T>> queue = new PriorityQueue<>(graph.getEdges());


        while (!queue.isEmpty() && s.size() / 2 < graph.getVertices().size() - 1) {
            Edge<T> e = queue.remove();
            if (!ds.find(e.getU()).equals(ds.find(e.getV()))) {
                s.add(e);
                Edge<T> newEdge = new Edge<>(e.getV(), e.getU(), e.getWeight());
                s.add(newEdge);
                ds.union(e.getU().getData(), e.getV().getData());
            }
        }

        if (s.size() / 2 != graph.getVertices().size() - 1) {
            return null;
        }

        return s;
    }
}
