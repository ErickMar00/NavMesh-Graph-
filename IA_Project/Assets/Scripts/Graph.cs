using System.Collections.Generic;
using UnityEngine;

// Clase que representa un nodo en el grafo
public class Node
{
    public string Id { get; private set; } // Identificador único del nodo
    public Node Parent { get; set; } // Nodo padre utilizado para reconstruir el camino

    public Node(string id)
    {
        Id = id;
    }
}

// Clase que representa una arista entre dos nodos en el grafo
public class Edge
{
    public Node A { get; private set; } // Nodo inicial de la arista
    public Node B { get; private set; } // Nodo final de la arista

    public Edge(Node a, Node b)
    {
        A = a;
        B = b;
    }
}

// Clase que representa el grafo y sus operaciones
public class Graph : MonoBehaviour
{
    public List<Node> Nodes { get; private set; } = new List<Node>(); // Lista de nodos en el grafo
    public List<Edge> Edges { get; private set; } = new List<Edge>(); // Lista de aristas en el grafo

    [Header("Nodos para búsqueda")]
    public string startNodeId; // Identificador del nodo de inicio para la búsqueda
    public string goalNodeId; // Identificador del nodo objetivo para la búsqueda

    void Start()
    {
        InitializeGraph(); // Inicializa el grafo con nodos y aristas predeterminados
        ExecuteSearches(); // Ejecuta las búsquedas DFS y BFS
    }

    // Inicializa el grafo con nodos y aristas predeterminados
    void InitializeGraph()
    {
        // Crear nodos con identificadores predeterminados
        string[] nodeIds = { "A", "B", "C", "D", "E", "F", "G", "H" };
        foreach (var id in nodeIds)
        {
            Nodes.Add(new Node(id));
        }

        // Crear aristas entre nodos predeterminados
        var edges = new List<(int, int)>
        {
            (0, 1), (1, 2), (1, 3), (0, 4), (4, 5), (4, 6), (4, 7)
        };

        // Añadir aristas a la lista de aristas
        foreach (var (a, b) in edges)
        {
            Edges.Add(new Edge(Nodes[a], Nodes[b]));
        }
    }

    // Ejecuta las búsquedas DFS y BFS, e imprime los resultados
    void ExecuteSearches()
    {
        Node startNode = FindNodeById(startNodeId); // Encuentra el nodo de inicio por su ID
        Node goalNode = FindNodeById(goalNodeId); // Encuentra el nodo objetivo por su ID

        if (startNode == null || goalNode == null)
        {
            Debug.LogError("Start Node or Goal Node not found!");
            return;
        }

        ResetGraph(); // Resetea el estado del grafo antes de la búsqueda
        Debug.Log("Iniciando DFS");
        if (DepthFirstSearch(startNode, goalNode))
        {
            Debug.Log("DFS completado. Camino encontrado:");
            PrintPath(goalNode); // Imprime el camino encontrado por DFS
        }
        else
        {
            Debug.Log("DFS completado. No se encontró camino.");
        }

        ResetGraph(); // Resetea el estado del grafo antes de la búsqueda
        Debug.Log("Iniciando BFS");
        if (BreadthFirstSearch(startNode, goalNode))
        {
            Debug.Log("BFS completado. Camino encontrado:");
            PrintPath(goalNode); // Imprime el camino encontrado por BFS
        }
        else
        {
            Debug.Log("BFS completado. No se encontró camino.");
        }
    }

    // Encuentra un nodo por su ID
    Node FindNodeById(string id)
    {
        return Nodes.Find(node => node.Id == id);
    }

    // Resetea el estado del grafo antes de cada búsqueda
    void ResetGraph()
    {
        foreach (var node in Nodes)
        {
            node.Parent = null;
        }
    }

    // Encuentra los nodos vecinos de un nodo dado
    List<Node> FindNeighbors(Node node)
    {
        List<Node> neighbors = new List<Node>();
        foreach (Edge edge in Edges)
        {
            if (edge.A == node)
                neighbors.Add(edge.B);
            else if (edge.B == node)
                neighbors.Add(edge.A);
        }
        return neighbors;
    }

    // Realiza una búsqueda en profundidad (DFS) en el grafo
    bool DepthFirstSearch(Node startNode, Node goalNode)
    {
        Stack<Node> stack = new Stack<Node>();
        HashSet<Node> visited = new HashSet<Node>();

        stack.Push(startNode);

        while (stack.Count > 0)
        {
            Node currentNode = stack.Pop();
            if (currentNode == goalNode)
            {
                return true; // Camino encontrado
            }

            visited.Add(currentNode);
            foreach (Node neighbor in FindNeighbors(currentNode))
            {
                if (!visited.Contains(neighbor))
                {
                    neighbor.Parent = currentNode;
                    stack.Push(neighbor);
                }
            }
        }
        return false; // Camino no encontrado
    }

    // Realiza una búsqueda en anchura (BFS) en el grafo
    bool BreadthFirstSearch(Node startNode, Node goalNode)
    {
        Queue<Node> queue = new Queue<Node>();
        HashSet<Node> visited = new HashSet<Node>();

        queue.Enqueue(startNode);
        visited.Add(startNode);

        while (queue.Count > 0)
        {
            Node currentNode = queue.Dequeue();
            if (currentNode == goalNode)
            {
                return true; // Camino encontrado
            }

            foreach (Node neighbor in FindNeighbors(currentNode))
            {
                if (!visited.Contains(neighbor))
                {
                    neighbor.Parent = currentNode;
                    queue.Enqueue(neighbor);
                    visited.Add(neighbor);
                }
            }
        }
        return false; // Camino no encontrado
    }

    // Imprime el camino desde el nodo inicial hasta el nodo objetivo
    void PrintPath(Node node)
    {
        Stack<Node> path = new Stack<Node>();
        while (node != null)
        {
            path.Push(node);
            node = node.Parent;
        }

        while (path.Count > 0)
        {
            Debug.Log(path.Pop().Id);
        }
    }
}

/*
 
Este codigo se hzio con ayuda de Ivan Medina :)
 
 */