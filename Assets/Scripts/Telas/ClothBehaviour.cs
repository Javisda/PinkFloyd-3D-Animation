using System.Collections.Generic;
using UnityEngine;

public class ClothBehaviour : MonoBehaviour
{
    // Constructor
    public ClothBehaviour()
    {
        this.Paused = true;
        this.TimeStep = 0.02f;
        this.Gravity = new Vector3(0.0f, -9.81f, 0.0f);
        this.IntegrationMethod = Integration.Symplectic;
        this.Mass = 10.0f;
        this.StiffnessTraction = 5.0f;
        this.StiffnessFlexion = 3.0f;
        this.NodeDamping = 0.1f;
        this.SpringDamping = 0.1f;
        this.setWind = false;
        this.substeps = 1;
    }


    // Integration Method
    public enum Integration
    {
        Explicit = 0,
        Symplectic = 1,
        Verlet = 2
    };


    #region InEditorVariables
    public List<GameObject> collideableSpheres;
    public List<GameObject> collideableCuboids;
    public WindZone[] winds;
    public GameObject[] fixers;

    [SerializeField] private Integration IntegrationMethod;
    [SerializeField] private bool Paused;
    [SerializeField] private float TimeStep;
    [SerializeField] private float substeps;
    [SerializeField] private float Mass;
    public float StiffnessTraction;
    public float StiffnessFlexion;
    public float NodeDamping;
    public float SpringDamping;
    public bool setWind;

    [Range(.0f, 100f)]
    [SerializeField] private float penaltyStiffnes;

    public Vector3 Gravity;
    #endregion


    #region OtherVariables
    public List<Node> nodes;
    public List<Spring> springs;

    private List<Edge> edges;
    private Vector3[] vertices;
    private Mesh mesh;

    private bool IsFixedToSomething;
    #endregion

    #region MonoBehaviour

    public void Start()
    {
        mesh = this.GetComponent<MeshFilter>().mesh;
        nodes = new List<Node>();
        springs = new List<Spring>();
        edges = new List<Edge>();

        // Detectamos si hay o no fixers asociados. En caso de que no activamos la variable que nos servirá para aleatorizar algunas propiedades de la tela ya que hará una caida libre, y esto le da mas gracia.
        if (fixers.Length > 0) { IsFixedToSomething = true; } else { IsFixedToSomething = false; }

        vertices = mesh.vertices;
        float massPerNode = Mass / vertices.Length;
        int[] triangles = mesh.triangles;

        // Add nodes
        for (int i = 0; i < vertices.Length; i++)
        {
            Vector3 pos;
            if (IsFixedToSomething)
            {
                pos = transform.TransformPoint(vertices[i]);
            }
            else { // En caso de que la tela vaya a caer en caida libre le damos una posicion de vérices un poco aleatoria para que no caiga plana.
                vertices[i].x += Random.Range(-0.5f, 0.5f);
                vertices[i].y += Random.Range(-0.5f, 0.5f);
                vertices[i].z += Random.Range(-0.5f, 0.5f);
                pos = transform.TransformPoint(vertices[i]);
            }

            nodes.Add(new Node(this, pos, massPerNode));
            if (!IsFixedToSomething){
                // Hacemos lo mismo para las verlocidades
                nodes[i].Vel.x += Random.Range(-5, 5);
                nodes[i].Vel.y += Random.Range(-5, 5);
                nodes[i].Vel.z += Random.Range(-5, 5);
            }
        }

        // Add fixers
        for (int i = 0; i < fixers.Length; i++)
        {
            Bounds bounds = fixers[i].GetComponent<Collider>().bounds;

            foreach (Node node in nodes)
            {
                if (bounds.Contains(node.Pos))
                {
                    // Add Fixed Attribute to Node
                    node.Fixed = true;

                    // Calculate Nodes-To-Fixer Offsets and set fixers
                    node.Offset = fixers[i].transform.InverseTransformPoint(node.Pos);
                    node.SetFixer(fixers[i].transform);
                }
            }
        }


        // -------------- CREACIÓN DE ARISTAS Y CREACION DE MUELLES DE FLEXIÓN Y TRACCIÓN -------------- //
        for (int i = 0; i < triangles.Length; i += 3)
        {
            edges.Add(new Edge(triangles[i], triangles[i + 1], triangles[i + 2]));
            edges.Add(new Edge(triangles[i + 1], triangles[i + 2], triangles[i]));
            edges.Add(new Edge(triangles[i + 2], triangles[i], triangles[i + 1]));
        }

        // Los ordenamos segun una clase comparadora que junta las asristas repetidas
        EdgeComparer comparer = new EdgeComparer();
        edges.Sort(comparer);

        // Para las aristas repetidas se crea un muelle con un stiffnes de flexión entre los vertexOther
        for (int i = 0; i < edges.Count - 1; i++)
        {
            //En caso de ser una arista repetida añadimos el muelle de flexión
            if ((edges[i].vertexA == edges[i + 1].vertexA) && (edges[i].vertexB == edges[i + 1].vertexB))
            {
                springs.Add(new Spring(this, nodes[edges[i].vertexOther], nodes[edges[i + 1].vertexOther], StiffnessFlexion, false));
            }
            else // En caso contrario el de traccion
            {
                springs.Add(new Spring(this, nodes[edges[i].vertexA], nodes[edges[i].vertexB], StiffnessTraction, true));
            }
        }
        // Añadimos el ultimo muelle ya que con el bucle anterior no se puede
        springs.Add(new Spring(this, nodes[edges[edges.Count - 1].vertexA], nodes[edges[edges.Count - 1].vertexB], StiffnessTraction, true));
        // ---------------------------------------------------------------------------------------------- //
    }


    public void Update()
    {
        if (Input.GetKeyUp(KeyCode.P))
            this.Paused = !this.Paused;

        // Key to update some values whiule running the simulation. Must be added more. Only works with mass and spring stiffness.
        if (Input.GetKeyDown(KeyCode.R))
            RefreshValues();

        //Procedure to update vertex positions
        mesh = this.GetComponent<MeshFilter>().mesh;
        vertices = new Vector3[mesh.vertexCount];

        // Actualizacion de las posiciones de los puntos que conforman el mallado
        for (int i = 0; i < nodes.Count; i++) {
            Vector3 pos;
            if (nodes[i].Fixed)
            {
                pos = nodes[i].Fixer.TransformPoint(nodes[i].Offset);
                nodes[i].Pos = pos;
            }
            else {
                pos = nodes[i].Pos;
            }
            vertices[i] = transform.InverseTransformPoint(pos);
        }

        mesh.vertices = vertices;
    }

    public void FixedUpdate()
    {
        if (this.Paused)
            return; // Not simulating

        // Substeps
        for (int i = 0; i < substeps; i++)
        {
            if (nodes[0].Mass < 0.0f) return; // We dont integrate things with infinite mass. Dont take Cloth Mass but any node mass so it is already applied to the nodes -> this.Mass != node.Mass

            if (setWind)
                ComputeWind();

            // Select integration method
            switch (this.IntegrationMethod)
            {
                case Integration.Explicit: this.eulerIntegrationMethod(); break;
                case Integration.Symplectic: this.eulerIntegrationMethod(); break;
                case Integration.Verlet: this.verletsIntegrationMethod(); break;
                default:
                    throw new System.Exception("[ERROR] Should never happen!");
            }
        }
    }

    private void eulerIntegrationMethod()
    {
        foreach (Node node in nodes)
        {
            // Clear the forces applied to the node
            node.ClearForce();

            // Compute Wind
            node.ComputeForces();

            // Check for Cuboid Collisions
            // Comment and uncomment to apply different methods
            //CheckCuboidCollisionMethod1(node);
            //CheckCuboidCollisionMethod2(node);
            CheckCuboidCollisionMethod3(node);

            node.Force = CheckSpheresCollisionMethod1(node);
        }

        foreach (Spring spring in springs)
        {
            spring.ComputeForces();
        }

        // Pos and Vel update
        foreach (Node node in nodes)
        {
            if (!node.Fixed && !node.cuboidCollision) // Se calcula la nueva posicion del nodo en caso de no pertenecer a un fixer y en caso de no estar chocando contra un cuboide (METODO 1 DE COLISION CON CUBOIDE).
            {
                if (Integration.Explicit == this.IntegrationMethod)
                {
                    node.Pos += (TimeStep) * node.Vel;
                    node.Vel += (TimeStep) / node.Mass * node.Force;
                }
                else if (Integration.Symplectic == this.IntegrationMethod)
                {
                    node.Vel += (TimeStep) / node.Mass * node.Force;
                    node.Pos += (TimeStep) * node.Vel;
                }
            }
        }

        foreach (Spring spring in springs)
        {
            spring.UpdateLength();
        }
    }

    private void verletsIntegrationMethod()
    {
        foreach (Node node in nodes)
        {
            // Clear the forces applied to the node
            node.ClearForce();

            // Compute Wind
            node.ComputeForces();

            // Check for Sphere Collisions
            node.Force = CheckSpheresCollisionMethod1(node);
        }
        foreach (Spring spring in springs)
        {
            spring.ComputeForces();
        }

        // Pos and Vel update
        foreach (Node node in nodes)
        {
            Vector3 temp = node.Pos;
            Vector3 nextPos = node.Pos + (node.Pos - node.PrevPos) + node.Force * 0.1f * 0.1f;
            if (!node.Fixed)
            {
                 node.Pos = nextPos;
            }

            node.PrevPos = temp;
        }

        foreach (Spring spring in springs)
        {
            spring.UpdateLength();
        }
    }
    #endregion

    private void RefreshValues()
    {
        // Mass
        float massPerNode = Mass / nodes.Count;
        foreach (Node node in nodes)
        {
            node.RefreshValues(massPerNode);
        }
        // Damping
        foreach (Spring spring in springs)
        {
            if (spring.Traction)
                spring.RefreshValues(StiffnessTraction);
            else
                spring.RefreshValues(StiffnessFlexion);
        }

        //TODO:
        //Implementar más valores
    }

    private Vector3 CheckSpheresCollisionMethod1(Node node)
    {
        Vector3 force = node.Force;
        // En caso de colisionar con un objeto se cambia la fuerza. En caso contrario se devuelve la misma que ya tenía.
        foreach (GameObject collideable in collideableSpheres)
        {
            // Fuerza de penalty
            float radius = collideable.GetComponent<SphereCollider>().radius * (collideable.transform.localScale.x * 1.25f);
            Vector3 dist = (node.Pos - collideable.transform.position);
            if (dist.magnitude < radius) 
            {
                dist.Normalize();
                force = dist * penaltyStiffnes;
            }
        }
        return force;
    }

    #region CuboidCollisionMethods
    private void CheckCuboidCollisionMethod1(Node node) {
        // Este metodo pone una variable auxiliar a true o false que hará que simplemente el nodo no actualice su posicion dentro del metodo de integracion.

        // Se hacen dos bucles para comprobar primero todos los que no colisionan, en vez de hacer un if else if dentro del foreach ya que eso solo nos permite colisionar con un cuboide como máximo.
        foreach (GameObject collideable in collideableCuboids)
        {
            if (!collideable.GetComponent<Collider>().bounds.Contains(node.Pos))
            {
                node.cuboidCollision = false;
            }
        }

        foreach (GameObject collideable in collideableCuboids)
        {
            if (collideable.GetComponent<Collider>().bounds.Contains(node.Pos))
            {
                node.cuboidCollision = true;
            }
        }
        
    }

    private void CheckCuboidCollisionMethod2(Node node)
    {
        // Este metodo trata de vencer la fuerza de la gravedad con una fuerza igual en sentido contrario, a la vez que pone a cero la velocidad acumulada.
        Vector3 force = new Vector3(0.0f, 9.81f, 0.0f);
        foreach (GameObject collideable in collideableCuboids)
        {
            if (collideable.GetComponent<Collider>().bounds.Contains(node.Pos))
            {
                node.Vel = Vector3.zero;
                node.Force = force;
            }
        }
    }

    private void CheckCuboidCollisionMethod3(Node node) {
        Vector3 force = node.Force;
        foreach (GameObject collideable in collideableCuboids) {
            if (collideable.GetComponent<Collider>().bounds.Contains(node.Pos)) {
                float closestXcoord;
                float closestYcoord;
                float closestZcoord;
                if (node.Pos.x > collideable.transform.position.x)
                    closestXcoord = node.Pos.x + (node.Pos.x - collideable.transform.position.x);
                else
                    closestXcoord = node.Pos.x + (collideable.transform.position.x - node.Pos.x);

                if(node.Pos.y > collideable.transform.position.y)
                    closestYcoord = node.Pos.y + (collideable.transform.position.y - node.Pos.y);
                else
                    closestYcoord = node.Pos.y + (node.Pos.y - collideable.transform.position.y);

                if (node.Pos.z > collideable.transform.position.z)
                    closestZcoord = node.Pos.z + (node.Pos.z - collideable.transform.position.z);
                else
                    closestZcoord = node.Pos.z + (collideable.transform.position.z - node.Pos.z);


                // TODO: Hay que mejorar el algoritmo ya que actualmente no se usa el closestX y closestZ ya que da malos resultados.
                Vector3 closestPoint = new Vector3(node.Pos.x, closestYcoord, node.Pos.z);
                Vector3 direction = closestPoint - node.Pos;
                direction.Normalize();
                node.Force = (-1) * direction * penaltyStiffnes;
            }
        }
    }
    #endregion


    #region WindComputation
    private Vector3 computeWindDirection()
    {
        // Hace una suma de los vectores directores de los vientos añadidos y los multiplica por la intensidad del viento para tratarlos con más o menos relevancia.
        Vector3 windDirection = new Vector3();
        for (int i = 0; i < winds.Length; i++)
        {
            windDirection += winds[i].transform.forward.normalized * winds[i].windMain;
        }
        return windDirection;
    }
    private void ComputeWind()
    {
        // Calculamos primero la direccion del viento resultante
        Vector3 windVelocity = computeWindDirection();

        // Cálculo de areas de triangulos. Se imprime una velocidad teniendo en cuenta la velocidad del triangulo, la del viento y el vector normal del propio triangulo. Se aplica mayor o menor fuerza dependiendo
        // del area y se reparte a cada nodo proporcionalmente
        for (int i = 0; i < mesh.triangles.Length; i += 3)
        {
            Vector3 v = nodes[mesh.triangles[i]].Pos - nodes[mesh.triangles[i + 1]].Pos;
            Vector3 w = nodes[mesh.triangles[i]].Pos - nodes[mesh.triangles[i + 2]].Pos;
            float area = Vector3.Cross(v, w).sqrMagnitude / 2;

            Vector3 triangleVelocity = (nodes[mesh.triangles[i]].Vel + nodes[mesh.triangles[i + 1]].Vel + nodes[mesh.triangles[i + 2]].Vel) / 3;
            Vector3 triangleNormal = Vector3.Cross(v, w);
            triangleNormal.Normalize();

            Vector3 force = 0.01f * area * (Vector3.Dot(triangleNormal, windVelocity - triangleVelocity) * triangleNormal);

            // Les atribuimos la fuerza resultante a los nodos
            nodes[mesh.triangles[i + 0]].WindForce = force / 3;
            nodes[mesh.triangles[i + 1]].WindForce = force / 3;
            nodes[mesh.triangles[i + 2]].WindForce = force / 3;
        }
    }
    #endregion
}

public class Node {
    public ClothBehaviour Cloth;

    public Vector3 Pos;
    public Vector3 PrevPos;
    public Vector3 Vel;
    public Vector3 Force;

    public Vector3 Offset;
    public Transform Fixer;

    public float Mass;
    public bool Fixed;

    public Vector3 WindForce;
    private Vector3 accForce;

    public bool cuboidCollision;

    public Node(ClothBehaviour c, Vector3 p, float massPortion)
    {
        Cloth = c;
        Pos = p;
        PrevPos = Pos;
        Vel = Vector3.zero;
        Fixed = false;
        Mass = massPortion;
        Offset = Vector3.zero;
        WindForce = Vector3.zero;
        accForce = Vector3.zero;
        cuboidCollision = false;
    }

    public void ComputeForces()
    {
        Force += Mass * (Cloth.Gravity + WindForce + accForce) - Cloth.NodeDamping * Vel;
    }

    public void RefreshValues(float m)
    {
        Mass = m;
    }

    public void SetFixer(Transform fixer)
    {
        Fixer = fixer;
    }

    public void ClearForce() {
        Force = Vector3.zero;
        accForce = Vector3.zero;
        if (!Cloth.setWind) {
            WindForce = Vector3.zero;
        }
    }

    public void AddForce(Vector3 f) {
        accForce += f;
    }
}

public class Spring {
    public ClothBehaviour Cloth;
    public Node nodeA, nodeB;

    public float Length0;
    public float Length;

    public float Stiffness;
    public float Damping;

    public bool Traction; // True if traction spring and false if Flexion type spring

    public Spring(ClothBehaviour c, Node a, Node b, float stiffness, bool edgeType)
    {
        Cloth = c;
        nodeA = a;
        nodeB = b;
        Stiffness = stiffness;
        Length0 = Length = (nodeA.Pos - nodeB.Pos).magnitude;
    }


    public void UpdateLength()
    {
        Length = (nodeA.Pos - nodeB.Pos).magnitude;
    }

    public void ComputeForces()
    {
        Vector3 u = nodeA.Pos - nodeB.Pos;
        u.Normalize();

        float modeloDeAmortiguamiento = Cloth.SpringDamping * Vector3.Dot(u, nodeA.Vel - nodeB.Vel);
        float stress = Stiffness * (Length - Length0) + modeloDeAmortiguamiento;
        Vector3 force = -stress * u;
        nodeA.Force += force;
        nodeB.Force -= force;
    }

    public void RefreshValues(float d)
    {
        Damping = d;
    }
}

public class Edge {
    public int vertexA;
    public int vertexB;
    public int vertexOther;

    public Edge(int a, int b, int o)
    {
        if (a < b)
        {
            this.vertexA = a;
            this.vertexB = b;
            this.vertexOther = o;
        }
        else if (b < a)
        {
            this.vertexA = b;
            this.vertexB = a;
            this.vertexOther = o;
        }
    }

    public override string ToString()
    {
        return "VertexA: " + vertexA + ". VertexB: " + vertexB + " VertexOther: " + vertexOther;
    }
}

public class EdgeComparer : IComparer<Edge>
{
    public int Compare(Edge a, Edge b)
    {
        if (a.vertexA > b.vertexA)
        {
            return 1;
        }
        else if (a.vertexA < b.vertexA)
        {
            return -1;
        }
        else if (a.vertexA == b.vertexA)
        {
            if (a.vertexB > b.vertexB)
            {
                return 1;
            }
            else if (a.vertexB < b.vertexB)
            {
                return -1;
            }
            else
            {
                return 0;
            }
        }
        return -1;
    }
}
