using System;
using System.Collections.Generic;
using System.Linq;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.InputSystem;

public static class PathfindingAlgorithm
{
    // TODO: Implement the pathfinding assignment here.
    // Find the lowest-cost path from start to goal using the map dimensions, wall costs and directed jumps exposed by IMapData.
    private static List<Vector2Int> vertices = new();
    private static Dictionary<Vector2Int, List<(Vector2Int edge, float cost)>> graph = new();




    public static List<Vector2Int> FindShortestPath(Vector2Int start, Vector2Int goal, IMapData mapData)
    {
        // Most of your solution should be implemented in this method.

        //Create graph from mapData if graph is empty:
        if (graph.Count == 0)
        {
            BuildGraph(mapData);
        }

        List<Vector2Int> path = null;

        //Mark the algorithm not to be used as comment and unmark the other.

        //BFS:
        //path = BFS(start, goal);

        //A*:
        path = AStarSearch(start, goal);

        if (path == null)
            Debug.LogWarning("FindShortestPath is not implemented OR no algorithm is active.");
        return path;
    }

    public static bool IsMovementBlocked(Vector2Int from, Vector2Int to, IMapData mapData)
    {
        if (mapData.TryGetJumpCost(from, to, out _))
            return false;

        int deltaX = to.x - from.x;
        int deltaY = to.y - from.y;

        if (Mathf.Abs(deltaX) + Mathf.Abs(deltaY) != 1)
            return true;

        if (deltaX != 0)
        {
            int wallX = deltaX > 0 ? to.x : from.x;
            float wallCost = mapData.GetVerticalWallCost(wallX, from.y);
            return float.IsPositiveInfinity(wallCost) || wallCost >= float.MaxValue;
        }

        int wallY = deltaY > 0 ? to.y : from.y;
        float horizontalWallCost = mapData.GetHorizontalWallCost(from.x, wallY);
        return float.IsPositiveInfinity(horizontalWallCost) || horizontalWallCost >= float.MaxValue;
    }

    private static List<Vector2Int> BFS(Vector2Int start, Vector2Int goal)
    {
        bool[] marked = new bool[graph.Count];
        int[] distTo = new int[graph.Count];
        int[] edgeTo = new int[graph.Count];
        Queue<Vector2Int> queue = new();

        for (int v = 0; v < graph.Count; v++)
            distTo[v] = int.MaxValue;

        int startIndex = vertices.IndexOf(start);
        distTo[startIndex] = 0;
        marked[startIndex] = true;

        queue.Enqueue(start);

        while (queue.Count > 0)
        {
            Vector2Int v = queue.Dequeue();
            foreach ((Vector2Int e, float) w in graph[v])
            {
                int wIndex = vertices.IndexOf(w.e);
                int vIndex = vertices.IndexOf(v);
                if (!marked[wIndex])
                {
                    edgeTo[wIndex] = vIndex;
                    distTo[wIndex] = distTo[vIndex] + 1;
                    marked[wIndex] = true;
                    queue.Enqueue(w.e);
                }
            }
        }

        int goalIndex = vertices.IndexOf(goal);
        if (marked[goalIndex])
        {
            List<Vector2Int> path = new();
            for (int i = goalIndex; distTo[i] != 0; i = edgeTo[i])
            {
                path.Add(vertices[i]);
            }

            path.Reverse();
            return path;
        }
        else
        {
            Debug.LogWarning("No path to goal.");
            return null;
        }
    }

    private class Cell
    {
        public int parent;
        public float f, g, h;

        public Cell()
        {
            parent = -1;
            f = float.MaxValue;
            g = float.MaxValue;
            h = float.MaxValue;
        }
    }

    private static List<Vector2Int> AStarSearch(Vector2Int start, Vector2Int goal)
    {
        bool[] closedList = new bool[graph.Count];

        Cell[] cells = new Cell[graph.Count];

        if (start == goal)
            return new();

        for (int i = 0; i < cells.Length; i++)
        {
            cells[i] = new Cell();
        }

        Vector2Int pos = start;
        int posIndex = vertices.IndexOf(pos);

        cells[posIndex].f = 0;
        cells[posIndex].g = 0;
        cells[posIndex].h = 0;
        cells[posIndex].parent = vertices.IndexOf(start);

        PriorityQueue openList = new();
        openList.Insert((posIndex, 0.0f));

        while (openList.Count > 0)
        {
            int index = openList.DelMax().nodeIndex;

            if (closedList[index])
                continue;

            closedList[index] = true;

            Vector2Int vertex = vertices[index];
            List<(Vector2Int edge, float cost)> edges = graph[vertex];

            for (int con = 0; con < edges.Count; con++)
            {
                if (!vertices.Contains(edges[con].edge))
                    continue;

                int conVertexIndex = vertices.IndexOf(edges[con].edge);

                if (edges[con].edge == goal)
                {
                    cells[conVertexIndex].parent = index;

                    //TracePath
                    Stack<Vector2Int> path = new();
                    int curIndex = vertices.IndexOf(goal);

                    while (cells[curIndex].parent != curIndex)
                    {
                        path.Push(vertices[curIndex]);

                        curIndex = cells[curIndex].parent;
                    }

                    path.Push(vertices[curIndex]);

                    return path.ToList();
                }

                if (!closedList[conVertexIndex])
                {
                    float gNew = cells[index].g + edges[con].cost;
                    float hNew = Mathf.Abs(vertex.x - goal.x) +
                        Mathf.Abs(vertex.y - goal.y);
                    float fNew = gNew + hNew;

                    if (cells[conVertexIndex].f == float.MaxValue || cells[conVertexIndex].f > fNew)
                    {
                        openList.Insert((conVertexIndex, fNew));

                        cells[conVertexIndex].f = fNew;
                        cells[conVertexIndex].g = gNew;
                        cells[conVertexIndex].h = hNew;
                        cells[conVertexIndex].parent = index;
                    }
                }
            }
        }

        Debug.LogWarning("Failed to find goal");
        return new();

    }

    private static void BuildGraph(IMapData mapData)
    {
        float inf = float.MaxValue;

        for (int y = 0; y < mapData.Height; y++)
        {
            for (int x = 0; x < mapData.Width; x++)
            {
                Vector2Int vertex = new(x, y);
                List<(Vector2Int to, float cost)> edges = new();
                if (x - 1 >= 0)//Left
                {
                    float cost = mapData.GetVerticalWallCost(x, y);

                    if (cost != inf)
                        edges.Add((new(x - 1, y), cost));
                }
                if (y + 1 < mapData.Height)//Up
                {
                    float cost = mapData.GetHorizontalWallCost(x, y + 1);

                    if (cost != inf)
                        edges.Add((new(x, y + 1), cost));

                }
                if (x + 1 < mapData.Width)//Right
                {
                    float cost = mapData.GetVerticalWallCost(x + 1, y);

                    if (cost != inf)
                        edges.Add((new(x + 1, y), cost));
                }
                if (y - 1 >= 0)//Down
                {
                    float cost = mapData.GetHorizontalWallCost(x, y);

                    if (cost != inf)
                        edges.Add((new(x, y - 1), cost));
                }
                if (mapData.HasJumpFrom(x, y))
                {
                    Vector2Int pos = new(x, y);
                    List<JumpConnection> jumps = mapData.GetJumpsFrom(pos);
                    foreach (JumpConnection j in jumps)
                    {
                        if (mapData.TryGetJumpCost(pos, j.To, out float cost))
                        {
                            edges.Add((j.To, cost));
                        }
                    }
                }

                vertices.Add(vertex);
                graph.Add(vertex, edges);

                //Lines for viewing created graph contents:
                //string message = vertex.ToString() + ": ";
                //foreach (Vector2Int e in edges)
                //    message += e + ", ";
                //Debug.Log(message);
            }
        }
    }

    private class PriorityQueue
    {
        private (int nodeIndex, float priority)[] priorityQueue;

        private int n = 0;

        public int Count => n;

        public PriorityQueue(int initialCapacity = 1)
        {
            priorityQueue = new (int, float)[initialCapacity];
        }

        //implemented from the book
        private void Resize(int capacity)
        {
            if (capacity < n)
                return;

            (int, float)[] temp = new (int, float)[capacity];
            for (int i = 1; i <= n; i++)
            {
                temp[i] = priorityQueue[i];
            }
            priorityQueue = temp;
        }

        public void Insert((int, float) x)
        {

            // double size of array if necessary
            if (n == priorityQueue.Length - 1)
                Resize(2 * priorityQueue.Length);

            priorityQueue[++n] = x;
            Swim(n);
        }

        public (int nodeIndex, float priority) DelMax()
        {
            if (n == 0)
                return (-1, -1);
            (int, float) max = priorityQueue[1];

            (priorityQueue[n], priorityQueue[1]) = (priorityQueue[1], priorityQueue[n]);
            n--;

            Sink(1);
            priorityQueue[n + 1] = (-1, -1);     // to avoid loitering and help with garbage collection
            if ((n > 0) && (n == (priorityQueue.Length - 1) / 4))
                Resize(priorityQueue.Length / 2);
            return max;
        }

        private void Swim(int k)
        {
            while (k > 1 && Less(k / 2, k))
            {
                (priorityQueue[k / 2], priorityQueue[k]) = (priorityQueue[k], priorityQueue[k / 2]);
                k /= 2;
            }
        }

        private void Sink(int k)
        {
            while (2 * k <= n)
            {
                int j = 2 * k;
                if (j < n && Less(j, j + 1)) j++;
                if (!Less(k, j)) break;
                (priorityQueue[k], priorityQueue[j]) = (priorityQueue[j], priorityQueue[k]);
                k = j;
            }
        }

        private bool Less(int i, int j)
        {

            return priorityQueue[i].priority > priorityQueue[j].priority;

        }
    }
}
