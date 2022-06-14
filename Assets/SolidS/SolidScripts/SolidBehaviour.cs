using System;
using System.Collections.Generic;
using System.Globalization;
using UnityEngine;


public class SolidBehaviour : MonoBehaviour
{

    public TextAsset nodesPosFile;
    public TextAsset tetrahedronsFile;
    


    // Debbuging purpose
    List<Vector3> normals;
    List<Vector3> pointsToNormal;
    private bool drawNormals = false;
    private bool drawNodes = false;
    private bool drawSprings = false;
    private bool drawTrianglesNormals = false;
    private bool drawWindForce = false;


    /// <summary>
    /// Default constructor. Zero all. 
    /// </summary>
    public SolidBehaviour()
    {
        this.Paused = true;
        this.TimeStep = 0.02f;
        this.Gravity = new Vector3(0.0f, -9.81f, 0.0f);
        this.IntegrationMethod = Integration.Symplectic;
        this.Mass = 10.0f;
        this.StiffnessTraction = 5.0f;
        this.NodeDamping = 0.1f;
        this.SpringDamping = 0.1f;
        this.setWind = false;
        this.Substeps = 1;
        this.ExternalForceIntensity = 10.0f;
    }


    /// <summary>
    /// Integration method.
    /// </summary>
    public enum Integration
    {
        Explicit = 0,
        Symplectic = 1
    };

    #region InEditorVariables
    public GameObject[] fixers;
    public List<GameObject> collideableSpheres;
    public List<GameObject> collideableCuboids;
    public WindZone[] winds;

    public Integration IntegrationMethod;
    public bool Paused;
    private float TimeStep;
    public float Substeps;
    public Vector3 Gravity;
    public float Mass;
    public float StiffnessTraction;
    public float NodeDamping;
    public float SpringDamping;
    public bool setWind;
    [Range(.0f, 100f)]
    [SerializeField] private float penaltyStiffnes;
    public bool ApplyForce;
    [Range(.0f, 1000f)]
    [SerializeField] private float ExternalForceIntensity;
    [HideInInspector] public Vector3 ExternalForceDirection;

    private List<SolidNode> nodes;
    private List<Vector3> nodesReset;
    private Dictionary<int, SolidSpring> springs;
    private List<Tetrahedron> tetrahedrons;
    private int[] vertexInfluencedByTetrahedronOfIndex;
    private List<Vector4> weigths;
    private Dictionary<int, Triangle> triangles;

    // Malla
    Mesh mesh;
    Vector3[] vertices;

    #endregion

    #region OtherVariables
    private bool canFix = true;
    #endregion

    #region MonoBehaviour

    public void Start()
    {
        nodes = new List<SolidNode>();
        nodesReset = new List<Vector3>();
        springs = new Dictionary<int, SolidSpring>();
        triangles = new Dictionary<int, Triangle>();
        tetrahedrons = new List<Tetrahedron>();
        weigths = new List<Vector4>();

        // PARSER
        // Soporte para ficheros en inglés.
        CultureInfo locale = new CultureInfo("en-US");
        // Parse vertices and create nodes
        string[] textPosNodes = nodesPosFile.text.Split(new string[] { " ", "\n", "\r" }, StringSplitOptions.RemoveEmptyEntries);
        int numNodes = int.Parse(textPosNodes[0]);
        for (int i = 0; i < numNodes; i++)
        {
            int id = int.Parse(textPosNodes[4 + (4 * i)]);
            float x = float.Parse(textPosNodes[5 + (4 * i)], locale);
            float y = float.Parse(textPosNodes[6 + (4 * i)], locale);
            float z = float.Parse(textPosNodes[7 + (4 * i)], locale);
            Vector3 initPos = transform.TransformPoint(new Vector3(x, y, z));
            SolidNode node = new SolidNode(this, initPos, NodeDamping, id);
            nodes.Add(node);
            nodesReset.Add(node.Pos);
        }


        // Add fixers
        for (int i = 0; i < fixers.Length; i++)
        {
            Bounds bounds = fixers[i].GetComponent<Collider>().bounds;

            foreach (SolidNode node in nodes)
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

        // Parse tetrahedron and create tetrahedrons
        string[] textIndicesTetrahedrons = tetrahedronsFile.text.Split(new string[] { " ", "\n", "\r" }, StringSplitOptions.RemoveEmptyEntries);
        int numTetrahedrons = int.Parse(textIndicesTetrahedrons[0]);
        for (int i = 0; i < numTetrahedrons; i++)
        {
            int node1 = int.Parse(textIndicesTetrahedrons[4 + (5 * i)]);
            int node2 = int.Parse(textIndicesTetrahedrons[5 + (5 * i)]);
            int node3 = int.Parse(textIndicesTetrahedrons[6 + (5 * i)]);
            int node4 = int.Parse(textIndicesTetrahedrons[7 + (5 * i)]);

            Tetrahedron tetrahedron; // Según el indice de inicio del fichero (cero o uno), hacemos una cosa u otra.
            if (int.Parse(textIndicesTetrahedrons[3]) == 1)
            {
                tetrahedron = new Tetrahedron(nodes[node1 - 1], nodes[node2 - 1], nodes[node3 - 1], nodes[node4 - 1], i, Mass);
                CreateTetrahedronSprings(node1 - 1, node2 - 1, node3 - 1, node4 - 1, tetrahedron.Volume / 6);
                CreateTriangles(tetrahedron);
            }
            else { 
                tetrahedron = new Tetrahedron(nodes[node1], nodes[node2], nodes[node3], nodes[node4], i, Mass);
                CreateTetrahedronSprings(node1, node2, node3, node4, tetrahedron.Volume / 6);
                CreateTriangles(tetrahedron);
            }
            tetrahedrons.Add(tetrahedron);
        }



        // Cargar mesh. Desde 3dsMax la malla ha de estar con la casilla "Flip YZ axis" desmarcada.
        mesh = GetComponentInChildren<MeshFilter>().mesh;
        vertices = mesh.vertices;

        // Debbuging purpose
        normals = new List<Vector3>();
        pointsToNormal = new List<Vector3>();


        vertexInfluencedByTetrahedronOfIndex = new int[vertices.Length];
        for (int i = 0; i < vertices.Length; i++) {
            Vector3 vertex = transform.TransformPoint(vertices[i]);

            //Por cada vértice se comprueba si está dentro de un tetraedro
            foreach (Tetrahedron tetrahedron in tetrahedrons) {
                if (isInside(vertex, tetrahedron))
                {
                    //Debug.Log("El vértice " + vertices[i] + " está dentro del tetraedro formado por los puntos P1: " + tetrahedron.a.Pos + ". P2: " + tetrahedron.b.Pos + ". P3: " + tetrahedron.c.Pos + ". P4: " + tetrahedron.d.Pos);
                    vertexInfluencedByTetrahedronOfIndex[i] = tetrahedron.Id;
                    // Cálculo de pesos
                    float w1 = Tetrahedron.CalculateTetrahedronVolume(vertex, tetrahedron.b.Pos, tetrahedron.c.Pos, tetrahedron.d.Pos) / tetrahedron.Volume;
                    float w2 = Tetrahedron.CalculateTetrahedronVolume(vertex, tetrahedron.a.Pos, tetrahedron.c.Pos, tetrahedron.d.Pos) / tetrahedron.Volume;
                    float w3 = Tetrahedron.CalculateTetrahedronVolume(vertex, tetrahedron.b.Pos, tetrahedron.a.Pos, tetrahedron.d.Pos) / tetrahedron.Volume;
                    float w4 = Tetrahedron.CalculateTetrahedronVolume(vertex, tetrahedron.b.Pos, tetrahedron.c.Pos, tetrahedron.a.Pos) / tetrahedron.Volume;
                    weigths.Add(new Vector4(w1, w2, w3, w4));
                    break; // Una vez asinado un tetraedro para de buscar en los demás
                }
                else {
                    //Debug.Log("El vértice " + vertices[i] + " NO está dentro del tetraedro formado por los puntos P1: " + tetrahedron.a.Pos + ". P2: " + tetrahedron.b.Pos + ". P3: " + tetrahedron.c.Pos + ". P4: " + tetrahedron.d.Pos);
                }
            }
        }
    }

    public void OnDrawGizmos()
    {
        if (drawNodes)
        {
            Gizmos.color = Color.blue;
            foreach (SolidNode n in nodes)
            {
                Gizmos.DrawSphere(n.Pos, 0.2f);//this.transform.localScale.x / mesh.vertices.Length * 2f);
            }
        }


        // Debugging purposes
        if (drawNormals)
        {
            Gizmos.color = Color.green;
            for (int i = 0; i < normals.Count; i++) {
                Gizmos.DrawLine(normals[i], pointsToNormal[i]);
            }
        }


        if (drawSprings) {
            Gizmos.color = Color.red;
            foreach(SolidSpring s in springs.Values)
            {
                Gizmos.DrawLine(s.nodeA.Pos, s.nodeB.Pos);
            }
        }


        if (drawTrianglesNormals)
        {
            foreach (Triangle t in triangles.Values)
            {
                Gizmos.color = Color.cyan;
                Gizmos.DrawLine(t.a.Pos, t.a.Pos + t.Normal);
                Gizmos.DrawLine(t.b.Pos, t.b.Pos + t.Normal);
                Gizmos.DrawLine(t.c.Pos, t.c.Pos + t.Normal);
            }
        }

        if (drawWindForce) {
            foreach (Triangle t in triangles.Values)
            {
                // Midpoints
                t.CalculateMidPoint();
                Gizmos.color = new Vector4(1.0f * t.Factor, 0.0f, 0.0f, 1.0f);
                Gizmos.DrawSphere(t.MidPoint, 0.2f);
            }
        }
    }

    public void Update()
    {
        // Show Gizmos
        if (Input.GetKeyUp(KeyCode.M))
            drawNormals = !drawNormals;
        if (Input.GetKeyUp(KeyCode.N))
            drawNodes = !drawNodes;
        if (Input.GetKeyUp(KeyCode.B))
            drawSprings = !drawSprings;
        if (Input.GetKeyUp(KeyCode.V))
            drawTrianglesNormals = !drawTrianglesNormals;
        if (Input.GetKeyUp(KeyCode.C))
            drawWindForce = !drawWindForce;

        // Other Controllers
        if (Input.GetKeyUp(KeyCode.P))
            this.Paused = !this.Paused;
        if (Input.GetKeyUp(KeyCode.X))
            canFix = !canFix;
        if (Input.GetKeyUp(KeyCode.R))
            ResetSolid();
        if (Input.GetKeyUp(KeyCode.Z))
            ApplyForce = true;
    }

    public void FixedUpdate()
    {
        if (this.Paused)
            return; // Not simulating

        // Substeps
        for (int i = 0; i < Substeps; i++)
        {
            // Select integration method
            switch (this.IntegrationMethod)
            {
                case Integration.Explicit: this.eulerIntegrationMethod(); break;
                case Integration.Symplectic: this.eulerIntegrationMethod(); break;
                default:
                    throw new System.Exception("[ERROR] Should never happen!");
            }
        }
    }

    private void eulerIntegrationMethod()
    {
        // -- VIENTO ---
        if (setWind) { 
            Vector3 windDirection = computeWindDirection();

            // Recorremos triangulos para el viento
            foreach(Triangle t in triangles.Values) {
                t.CalculateNormal(); // Actualizamos la normal

                float angle = Vector3.Angle(t.Normal, windDirection * (-1)); // Negamos en vector del viento para que salga en la misma direccion a la cara del triangulo
                if (angle < 90.0f && 0.0f < angle) {
                    float factor;
                    if (angle >= 85.0f) // Para evitar valores que se acercan mucho a 0 con el cálculo del coseno, ponemos a 0 directamente para un valor más temprano
                        factor = 0;
                    else
                        factor = Mathf.Cos((angle * (float)Math.PI)/180);

                    // Area
                    Vector3 v = t.a.Pos - t.b.Pos;
                    Vector3 w = t.a.Pos - t.c.Pos;
                    float area = Vector3.Cross(v, w).sqrMagnitude / 2;
                    Vector3 force = windDirection * factor * (area * 0.01f);

                    // Apply forces
                    t.a.WindForce = force;
                    t.b.WindForce = force;
                    t.c.WindForce = force;
                    t.Factor = factor;
                }
            }
        }


        foreach (SolidNode node in nodes)
        {
            node.ClearForce();

            // Apply External Force
            if (ApplyForce)
            {
                ApplyExternalForce(node);
            }

            // Calcula las fisicas
            if (node.Fixed == false)
            {
                if (canFix)
                    CheckSolidsToRest(node);

                node.ComputeForces();
                CheckCuboidCollisions(node);
                node.Force = CheckSpheresCollisions(node);
            }

            // Quita los fixers en caso de indicarselo
            if (!canFix)
                RemoveFixer(node);
        }
        // Ponemos a false inmediatamente para que solo se haga en un ciclo de ejecucion
        ApplyForce = false;

        foreach (SolidSpring spring in springs.Values)
        {
            spring.ComputeForces();
        }

        // Se calculan las posiciones
        foreach (SolidNode node in nodes)
        {
            if (!node.Fixed)
            {
                if (Integration.Explicit == this.IntegrationMethod)
                {
                    node.Pos += TimeStep * node.Vel;
                    node.Vel += TimeStep / node.Mass * node.Force;
                }
                else if (Integration.Symplectic == this.IntegrationMethod)
                {
                    node.Vel += TimeStep / node.Mass * node.Force;
                    node.Pos += TimeStep * node.Vel;
                }
            }
            else {
                // Actualizacion de nodos respecto a la posicion de sus fixers
                node.Pos = node.Fixer.TransformPoint(node.Offset);
            }
        }

        foreach (SolidSpring spring in springs.Values)
        {
            spring.UpdateLength();
        }

        UpdateMesh();
        mesh.RecalculateNormals();
    }
    #endregion

    private void CreateTetrahedronSprings(int i1, int i2, int i3, int i4, float Volume) {

        // Create Springs in every Tetrahedron
        CreateSpring(nodes[i1], nodes[i2], Volume);
        CreateSpring(nodes[i1], nodes[i3], Volume);
        CreateSpring(nodes[i1], nodes[i4], Volume);
        CreateSpring(nodes[i2], nodes[i3], Volume);
        CreateSpring(nodes[i2], nodes[i4], Volume);
        CreateSpring(nodes[i3], nodes[i4], Volume);
    }

    private void CreateSpring(SolidNode a, SolidNode b, float Volume) {
        int hash = 23;
        hash = hash * 31 + a.Id;
        hash = hash * 31 + b.Id;

        if (!springs.ContainsKey(hash))
        {
            SolidSpring spring = new SolidSpring(this, a, b, SpringDamping, StiffnessTraction, Volume / 6);
            springs.Add(hash, spring);
        }
    }

    private void CreateTriangles(Tetrahedron t) {
        CreateTriangle(t.a, t.b, t.c);
        CreateTriangle(t.c, t.d, t.a);
        CreateTriangle(t.a, t.d, t.b);
        CreateTriangle(t.b, t.d, t.c);
    }
    private void CreateTriangle(SolidNode a, SolidNode b, SolidNode c)
    {
        // Order from min to max
        int min, mid, max;
        if (a.Id < b.Id && a.Id < c.Id)
        {
            min = a.Id;
            if (b.Id < c.Id)
            {
                mid = b.Id; max = c.Id;
            }
            else
            {
                mid = c.Id; max = b.Id;
            }
        }
        else if (b.Id < a.Id && b.Id < c.Id)
        {
            min = b.Id;
            if (a.Id < c.Id)
            {
                mid = a.Id; max = c.Id;
            }
            else
            {
                mid = c.Id; max = a.Id;
            }
        }
        else {
            min = c.Id;
            if (a.Id < b.Id)
            {
                mid = a.Id; max = b.Id;
            }
            else
            {
                mid = b.Id; max = a.Id;
            }
        }


        string hashAux = min.ToString() + mid.ToString() + max.ToString();
        int hash = Int32.Parse(hashAux);

        if (!triangles.ContainsKey(hash))
        {
            Triangle triangle = new Triangle(a, b, c);
            triangles.Add(hash, triangle);
        }
        else {
            // Si el triangulo ya existe, significa que el que se mete está repetido. Ergo, es interior. Ergo no nos interesa ni si quiera tenerlo por eso lo eliminamos.
            triangles.Remove(hash);
        }
    }

    private bool isInside(Vector3 pos, Tetrahedron t) {
        Vector3 p1 = t.a.Pos;
        Vector3 p2 = t.b.Pos;
        Vector3 p3 = t.c.Pos;
        Vector3 p4 = t.d.Pos;
                                                             // El orden de estos puntos es de gran importancia. No se pueden poner de forma arbitraria. Esta es una de las posibles permutaciones
        if (CompruebaPosDirNormal(pos, p1, p2, p3) &&        // p1, p2, p3     
            CompruebaPosDirNormal(pos, p3, p4, p1) &&        // p3, p4, p1     
            CompruebaPosDirNormal(pos, p1, p4, p2) &&        // p1, p4, p2     
            CompruebaPosDirNormal(pos, p2, p4, p3)){         // p2, p4, p3     
            return true; // El punto está contenido dentro del tetraedro
        }


        return false;
    }

    private bool CompruebaPosDirNormal(Vector3 pos, Vector3 p1, Vector3 p2, Vector3 p3) {
        // Se comprueba si el punto está en la direccion normal del plano, es decir, hacia dentro del tetraedro
        Vector3 p2p1 = p2 - p1;
        Vector3 p3p1 = p3 - p1;
        Vector3 normal = Vector3.Cross(p2p1, p3p1);

        // Debugging purposes
        pointsToNormal.Add(pos);
        // Las normales serán proporcionales al escalado y el numero de resolucion de la malla, para que cada normal de cada objeto sea diferente.
        normals.Add(pos + (normal.normalized / mesh.vertices.Length * 10f * this.transform.localScale.x));

        // Ecuacion del plano: Vector3(PuntoGenérico - PuntoCualquiera(p1, p2 o p3)) * normal -> producto escalar. Ejemplo: ((x - p1.x) * normal.x) + ((y - p1.y) * normal.y) + ((z - p1.z) * normal.z) = 0
        // Comprobamos sustituyendo ya en la ecucion del plano el punto que nos interesa -> pos
        float resultado = ((pos.x - p1.x) * normal.x) + ((pos.y - p1.y) * normal.y) + ((pos.z - p1.z) * normal.z);

        if (resultado >= -0.0001) { // Está por encima de el plano. Es decir, para nuestro caso, en la direccion en la que podría estár dentro del tetraedro. El -0.001 es un margen de error.
            return true;
        }

        return false;
    }

    private void UpdateMesh()
    {
        Vector3[] newVertexPositions = new Vector3[vertices.Length];
        for (int i = 0; i < vertices.Length; i++) {
            int id = vertexInfluencedByTetrahedronOfIndex[i];
            Tetrahedron t = tetrahedrons[id];
            Vector3 vertexAux = Vector3.zero;

            vertexAux += weigths[i][0] * (t.a.Pos);
            vertexAux += weigths[i][1] * (t.b.Pos);
            vertexAux += weigths[i][2] * (t.c.Pos);
            vertexAux += weigths[i][3] * (t.d.Pos);


            vertices[i] = newVertexPositions[i] = transform.InverseTransformPoint(vertexAux);
        }
        mesh.vertices = newVertexPositions;
    }


    #region WindComputation
    private Vector3 computeWindDirection()
    {
        // Hace una suma de los vectores directores de los vientos a�adidos y los multiplica por la intensidad del viento para tratarlos con m�s o menos relevancia.
        Vector3 windDirection = new Vector3();
        for (int i = 0; i < winds.Length; i++)
        {
            windDirection += winds[i].transform.forward.normalized * winds[i].windMain;
        }
        return windDirection;
    }
    #endregion

    private void CheckSolidsToRest(SolidNode n)
    {
        foreach (GameObject g in fixers) {
            if (g.GetComponent<Collider>().bounds.Contains(n.Pos))
            {
                n.Fixed = true;
                n.Offset = g.transform.InverseTransformPoint(n.Pos);
                n.SetFixer(g.transform);
                break;
            }
        }
    }

    private void RemoveFixer(SolidNode n) {
         n.Fixed = false;
         n.Offset = Vector3.zero;
         n.SetFixer(null);
    }

    #region Collisions
    private Vector3 CheckSpheresCollisions(SolidNode node)
    {
        Vector3 force = node.Force;
        // En caso de colisionar con un objeto se cambia la fuerza. En caso contrario se devuelve la misma que ya ten�a.
        foreach (GameObject collideable in collideableSpheres)
        {
            // Fuerza de penalty
            if (collideable.GetComponent<SphereCollider>().bounds.Contains(node.Pos)) { 
                Vector3 dir = (node.Pos - collideable.transform.position);

                dir.Normalize();
                force = dir * penaltyStiffnes;
            }
        }
        return force;
    }


    private void CheckCuboidCollisions(SolidNode node)
    {
        foreach (GameObject collideable in collideableCuboids)
        {
            if (collideable.GetComponent<Collider>().bounds.Contains(node.Pos - collideable.transform.InverseTransformPoint(node.Pos)))
            {
                Vector3 direction = collideable.transform.up;
                node.Force = (-1) * direction * penaltyStiffnes;
            }
        }
    }
    #endregion


    private void ResetSolid() {
        for(int i = 0; i < nodes.Count; i++) {
            nodes[i].Fixed = false;
            nodes[i].ClearForce();
            nodes[i].Pos = nodesReset[i];
        }
    }


    private void ApplyExternalForce(SolidNode n) {
        n.Force = ExternalForceIntensity * ExternalForceDirection;
    }
}





// ------------------------------------------------------------------------------------------ //
public class SolidNode
{
    public SolidBehaviour Solid;

    public Vector3 Pos;
    public Vector3 Vel;
    public Vector3 Force;

    public Vector3 Offset;
    public Transform Fixer;

    public float Mass;
    public bool Fixed;
    public float Damping;
    public Vector3 WindForce;

    public int Id;

    public SolidNode(SolidBehaviour m, Vector3 p, float damping, int id)
    {
        Solid = m;
        Pos = p;
        Vel = Vector3.zero;
        Fixed = false;
        Damping = damping;
        WindForce = Vector3.zero;
        Offset = Vector3.zero;
        Id = id;
    }

    public void ComputeForces()
    {
        Force += Mass * (Solid.Gravity + WindForce) - Damping * Vel;
    }


    public void RefreshValues(float m)
    {
        Mass = m;
    }

    public void SetFixer(Transform fixer)
    {
        Fixer = fixer;
    }

    public void AddMass(float m)
    {
        Mass += m;
    }

    public void ClearForce()
    {
        Force = Vector3.zero;
        if (!Solid.setWind)
        {
            WindForce = Vector3.zero;
        }
    }
}






// ------------------------------------------------------------------------------------------ //

public class SolidSpring
{
    public SolidBehaviour Solid;
    public SolidNode nodeA, nodeB;

    public float Length0;
    public float Length;

    public float Stiffness;
    public float Damping;

    public bool Traction; // True if traction spring and false if Flexion type spring
    public float Volume;

    public SolidSpring(SolidBehaviour m, SolidNode a, SolidNode b, float damping, float stiffness, float volume)
    {
        Solid = m;
        nodeA = a;
        nodeB = b;
        Stiffness = stiffness;
        Damping = damping;
        Length0 = Length = (nodeA.Pos - nodeB.Pos).magnitude;
        Volume = volume;
    }


    public void UpdateLength()
    {
        Length = (nodeA.Pos - nodeB.Pos).magnitude;
    }

    public void ComputeForces()
    {
        // Manera nº2 teniendo en cuenta la densidad de rigidez
        Vector3 force = ((-1) * Volume * Stiffness * (Length - Length0) * (nodeA.Pos - nodeB.Pos)) / (Length0 * Length0 * Length);

        nodeA.Force += force;
        nodeB.Force -= force;
    }

    public void RefreshValues(float d)
    {
        Damping = d;
    }
}

// ------------------------------------------------------------------------------------------ //

public class Tetrahedron
{
    public SolidNode a, b, c, d;
    public float Volume;
    public float Mass;
    public float MassDensity;


    public int Id;
    public Tetrahedron(SolidNode a, SolidNode b, SolidNode c, SolidNode d, int id, float md)
    {
        this.a = a;
        this.b = b;
        this.c = c;
        this.d = d;
        this.Id = id;
        CalculateTetrahedronVolume();
        this.MassDensity = md;
        SetMass();
    }

    public void SetMass() {
        this.Mass = this.Volume * this.MassDensity;
        float massNode = this.Mass / 4;
        a.Mass = massNode;
        b.Mass = massNode;
        c.Mass = massNode;
        d.Mass = massNode;
    }

    private void CalculateTetrahedronVolume()
    {
        Vector3 u = b.Pos - a.Pos;
        Vector3 v = c.Pos - a.Pos;
        Vector3 w = d.Pos - a.Pos;
        Vector3 uxv = Vector3.Cross(u, v);
        this.Volume = Math.Abs((Vector3.Dot(w, uxv)) / 6);
    }
    public static float CalculateTetrahedronVolume(Vector3 p1, Vector3 p2, Vector3 p3, Vector3 p4) {
        // Producto vectorial de dos vectores. El resultado se hace el producto escalar con el otro vector. Se divide entre 6.
        // (w * |u x v|) / 6
        Vector3 u = p2 - p1;
        Vector3 v = p3 - p1;
        Vector3 w = p4 - p1;
        Vector3 uxv = Vector3.Cross(u, v);
        float volume = Math.Abs((Vector3.Dot(w, uxv)) / 6);
        return volume;
    }
}


public class Triangle {

    public SolidNode a;
    public SolidNode b;
    public SolidNode c;
    public Vector3 Normal;
    public Vector3 MidPoint;
    public float Factor;
    public Triangle(SolidNode a, SolidNode b, SolidNode c) {
        this.a = a;
        this.b = b;
        this.c = c;
        CalculateNormal();
    }

    public void CalculateNormal() {
        Vector3 p2p1 = b.Pos - a.Pos;
        Vector3 p3p1 = c.Pos - a.Pos;
        Vector3 normal = -Vector3.Cross(p2p1, p3p1);
        normal.Normalize();
        this.Normal = normal;
    }

    // For debugging purposes
    public void CalculateMidPoint() {
        this.MidPoint = new Vector3((a.Pos.x + b.Pos.x + c.Pos.x) / 3, (a.Pos.y + b.Pos.y + c.Pos.y) / 3, (a.Pos.z + b.Pos.z + c.Pos.z) / 3);
    }
}