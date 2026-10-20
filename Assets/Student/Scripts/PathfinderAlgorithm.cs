using System.Collections.Generic;
using System.Linq;
using Unity.VisualScripting;
using UnityEngine;

public static class PathfindingAlgorithm
{
    // TODO: Implement the pathfinding assignment here.
    // Find the lowest-cost path from start to goal using the map dimensions, wall costs and directed jumps exposed by IMapData.
    private static List<Vector2Int> vertices = new();
    private static Dictionary<Vector2Int, List<Vector2Int>> graph = new();




    public static List<Vector2Int> FindShortestPath(Vector2Int start, Vector2Int goal, IMapData mapData)
    {
        // Most of your solution should be implemented in this method.

        //Create graph from mapData if graph is empty:
        if (graph.Count == 0)
        {
            for (int y = 0; y < mapData.Height; y++)
            {
                for (int x = 0; x < mapData.Width; x++)
                {
                    Vector2Int vertex = new(x, y);
                    List<Vector2Int> edges = new();
                    if (x - 1 >= 0 && !mapData.HasVerticalWall(x, y))//Left
                        edges.Add(new(x - 1, y));
                    if (y + 1 < mapData.Height && !mapData.HasHorizontalWall(x, y + 1))//Up
                        edges.Add(new(x, y + 1));
                    if (x + 1 < mapData.Width && !mapData.HasVerticalWall(x + 1, y))//Right
                        edges.Add(new(x + 1, y));
                    if (y - 1 >= 0 && !mapData.HasHorizontalWall(x, y))//Down
                        edges.Add(new(x, y - 1));

                    vertices.Add(vertex);
                    graph.Add(vertex, edges);
                    //string message = vertex.ToString() + ": ";
                    //foreach (Vector2Int e in edges)
                    //    message += e + ", ";
                    //Debug.Log(message);
                }
            }
        }

        //BFS:
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
            foreach (Vector2Int w in graph[v])
            {
                int wIndex = vertices.IndexOf(w);
                int vIndex = vertices.IndexOf(v);
                if (!marked[wIndex])
                {
                    edgeTo[wIndex] = vIndex;
                    distTo[wIndex] = distTo[vIndex] + 1;
                    marked[wIndex] = true;
                    queue.Enqueue(w);
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

        Debug.LogWarning("FindShortestPath is not implemented.");
        return null;
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
}
