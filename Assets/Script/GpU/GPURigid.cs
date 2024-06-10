using System.Collections;
using System.Collections.Generic;
using ExporterImporter;
using Octree;
using PBD;
using UnityEngine;
using UnityEngine.Rendering;

public class GPURigid : MonoBehaviour
{
    public enum MyModel
    {
        Cube,
        IcoSphere_low,
        Torus,
        Bunny,
        Armadillo,
    };

    [Header("3D model")]
    public MyModel model;
    [HideInInspector]
    private string modelName;

    [Header("Obj Parameters")]
    public int numberOfObjects = 1;
    public float invMass = 1.0f;
    public float dt = 0.01f; // have to devide by 20
    public Vector3 gravity = new Vector3(0f, -9.81f, 0f);

    [Header("Import CSV")]
    public string csv_file = "object_positions.csv";

    [Header("Collision")]
    public GameObject floor;

    [Header("Rendering Paramenter")]
    public ComputeShader computeShader;
    public Shader renderingShader;
    public Color matColor;

    [HideInInspector]
    private Material material;
     private ComputeShader computeShaderObj;

    [HideInInspector]
    private int nodeCount;
    private int triCount; // size of triangle
    
    //for render
    private ComputeBuffer vertsBuff = null;
    private ComputeBuffer triBuffer = null;
   //for compute shader
    private ComputeBuffer positionsBuffer;
    private ComputeBuffer velocitiesBuffer ;
    private ComputeBuffer triangleBuffer;
    private ComputeBuffer triangleIndicesBuffer;
    private ComputeBuffer floorBBBuffer ;
    private ComputeBuffer floorPositionsBuffer;
    private ComputeBuffer bbMinMaxBuffer ;
    private ComputeBuffer objectIndexBuffer;
    private ComputeBuffer floorCollisionResultBuffer ;
    private ComputeBuffer bbOctreeBuffer;
    private ComputeBuffer pairIndexLv2Buffer;
    private ComputeBuffer collisionResultsBuffer;

    List<Triangle> triangles = new List<Triangle>();
    private int computeVerticesNormal; // for rendering purpose 
    private int updatePosKernel;
    private int findFloorMinMaxKernel;
    private int findBBMinMaxKernel; 
    private int updateReverseVelocityKernel;
    private int implementOctreeKernel;
    private int checkCollisionOctreeKernel;
    private int removeCollisionResultsKernel;

    private Vector3[] Positions;
    private Vector3[] Velocities;
    private Vector2Int[] indicies;
    struct vertData
    {
        public Vector3 pos;
        public Vector2 uvs;
        public Vector3 norms;
    };
    int[] triArray;
    vertData[] vDataArray;
    // octree
    private OctreeData[] bbOctree;
    private List<PairData> pairIndexL0 = new List<PairData>();
    private List<PairData> pairIndexL1 = new List<PairData>();
    private List<PairData> pairIndexL2 = new List<PairData>();
    private readonly int octree_size = 73;
    private int[] collisionResults;

     // collidable pair
     private List<int> collidableIndex = new List<int>();

     [Header("Debug Mode")]
    public bool debugData = true;
    public bool debugLv0 = true; 
    public bool debugLv1 = true; 
    public bool debugLv2 = true; 

    void Start()
    {
        material = new Material(renderingShader); // new material for difference object
        material.color = matColor; //set color to material
        computeShaderObj = Instantiate(computeShader); // to instantiate the compute shader to be use with multiple object

        SelectModelName();
        AddOctreePairIndex();
        SetupMeshData();
        SetupShader();
        SetBuffData();
        SetupComputeBuffer();
        SetupKernel();
        SetupComputeShader();
    }

    void SelectModelName()
    {
        switch (model)
        {
            case MyModel.Cube: modelName = "33cube.1"; break;
            case MyModel.IcoSphere_low: modelName = "icosphere_low.1"; break;
            case MyModel.Torus: modelName = "torus.1"; break;
            case MyModel.Bunny: modelName = "bunny.1"; break;
            case MyModel.Armadillo: modelName = "Armadillo.1"; break;
        }
    }
    
    int calculateStartIndex(int object_index, int level)
    {
        if (level == 1) return object_index * 73 + 1;
        if (level == 2) return object_index * 73 + 9;
        return object_index * 73 ;
    }

    int calculateEndIndex(int object_index, int level)
    {
        if (level == 1) return object_index * 73 + 8;
        if (level == 2) return object_index * 73 + 73;
        return object_index * 73 ;
    }

    private void AddOctreePairIndex()
    {
        for (int i = 0; i < numberOfObjects; i++)
            {
                var l2_st_idx = calculateStartIndex(i, 2);
                var l2_end_idx = calculateEndIndex(i, 2);

                for (int j = 0; j < numberOfObjects; j++)
                {
                    if (i == j) break;
                
                    var j_l2_end_idx = calculateEndIndex(j, 2);
                    var j_l2_st_idx = calculateStartIndex(j, 2);

                    for (int n = l2_st_idx; n < l2_end_idx; n++)
                    {
                        for (int m = j_l2_st_idx; m < j_l2_end_idx; m++)
                        {
                            pairIndexL2.Add( new PairData
                            {
                                i1 = n,
                                i2 = m
                            });
                        }
                    }
                }
            }
    }

    private void SetupMeshData()
    {
        var number = numberOfObjects;
        print(Application.dataPath);
        string filePath = Application.dataPath + "/TetModel/";
        LoadTetModel.LoadData(filePath + modelName, gameObject);
        List<List<string>> csvData = ExporterAndImporter.ReadCSVFile(csv_file);
        // if(number > csvData.Count) // exit app
        // {

        //     throw new Exception("cc"); 
        //     //  UnityEditor.EditorApplication.isPlaying = false;
            
        // } 
      
        indicies = new Vector2Int[number];
        int st_index = 0;

        var _Positions = LoadTetModel.positions.ToArray();            
        var _triangles = LoadTetModel.triangles;
        var _triArray = LoadTetModel.triangleArr.ToArray();

        // print($" {_Positions.Length}");

        Positions = new Vector3[number * LoadTetModel.positions.Count];
        Velocities = new Vector3[number * LoadTetModel.positions.Count];
        triangles = new List<Triangle>(new Triangle[number * LoadTetModel.triangles.Count]);
        triArray = new int[number * LoadTetModel.triangleArr.Count];

         for(int i=0;i<number;i++){
            int PosOffset = i * LoadTetModel.positions.Count;
            
            List<string> column = csvData[i];
            float x = float.Parse(column[0]);
            float y = float.Parse(column[1]);
            float z = float.Parse(column[2]);
            Vector3 Offset = new Vector3(x, y, z);
            
            for(int j=0;j<LoadTetModel.positions.Count;j++){                
                Positions[j+PosOffset] = _Positions[j] + Offset;
            }

            int TriOffset = i * LoadTetModel.triangles.Count;
            for(int j=0;j<LoadTetModel.triangles.Count;j++){
                var t = _triangles[j];
                triangles[j+TriOffset] = new Triangle(t.vertices[0] + PosOffset, t.vertices[1] + PosOffset, t.vertices[2] + PosOffset);
            }

            int TriArrOffset = i * LoadTetModel.triangleArr.Count;
            for(int j=0;j<LoadTetModel.triangleArr.Count;j++){
                triArray[j+TriArrOffset] = _triArray[j] + PosOffset;
            }

            indicies[i] = new Vector2Int(st_index, st_index + LoadTetModel.positions.Count);
            st_index += LoadTetModel.positions.Count;
        }        

        nodeCount = Positions.Length;
        triCount = triangles.Count; //
        vDataArray = new vertData[nodeCount];

        for (int i = 0; i < nodeCount; i++)
        {
            vDataArray[i] = new vertData();
            vDataArray[i].pos = Positions[i];
            vDataArray[i].norms = Vector3.zero;
            vDataArray[i].uvs = Vector3.zero;
        }

        int triBuffStride = sizeof(int);
        triBuffer = new ComputeBuffer(triArray.Length,
            triBuffStride, ComputeBufferType.Default);


        int vertsBuffstride = 8 * sizeof(float);
        vertsBuff = new ComputeBuffer(vDataArray.Length,
            vertsBuffstride, ComputeBufferType.Default);
        LoadTetModel.ClearData();

        // print("node count: " + nodeCount);
        // print("tri  count: " + triCount);

    }

     private void SetupShader()
    {
        material.SetBuffer(Shader.PropertyToID("vertsBuff"), vertsBuff);
        material.SetBuffer(Shader.PropertyToID("triBuff"), triBuffer);
    }

     private void SetBuffData()
    {
        vertsBuff.SetData(vDataArray);
        triBuffer.SetData(triArray);

        Vector3 translation = transform.position;
        Vector3 scale = this.transform.localScale;
        Quaternion rotationeuler = transform.rotation;
        Matrix4x4 trs = Matrix4x4.TRS(translation, rotationeuler, scale);
        material.SetMatrix("TRSMatrix", trs);
        material.SetMatrix("invTRSMatrix", trs.inverse);
    }

    private void SetupComputeBuffer()
    {
        bbOctree = new OctreeData[numberOfObjects * octree_size];
        collisionResults = new int[numberOfObjects * octree_size];

        positionsBuffer = new ComputeBuffer(nodeCount, sizeof(float) * 3);
        positionsBuffer.SetData(Positions);

        velocitiesBuffer = new ComputeBuffer(nodeCount, sizeof(float) * 3);
        velocitiesBuffer.SetData(Velocities);

        List<MTriangle> initTriangle = new List<MTriangle>();  //list of triangle cooresponding to node 
        List<int> initTrianglePtr = new List<int>(); //contain a group of affectd triangle to node
     
        Dictionary<int, List<int>> nodeTriangles = new Dictionary<int, List<int>>();
        for (int triIndex = 0; triIndex < triangles.Count; triIndex++)
        {
            Triangle tri = triangles[triIndex];
            for (int vertexIndex = 0; vertexIndex < 3; vertexIndex++)
            {
                int vertex = tri.vertices[vertexIndex];
                if (!nodeTriangles.ContainsKey(vertex))
                {
                    nodeTriangles[vertex] = new List<int>();
                }
                nodeTriangles[vertex].Add(triIndex);
            }
        }
        initTrianglePtr.Add(0);
        for (int i = 0; i < nodeCount; i++)
        {
            if (nodeTriangles.TryGetValue(i, out List<int> triangleIndexes))
            {
                foreach (int triIndex in triangleIndexes)
                {
                    Triangle tri = triangles[triIndex];
                    MTriangle tmpTri = new MTriangle { v0 = tri.vertices[0], v1 = tri.vertices[1], v2 = tri.vertices[2] };
                    initTriangle.Add(tmpTri);
                }
            }
            initTrianglePtr.Add(initTriangle.Count);
        }

        // print(initTrianglePtr.Count);

        triangleBuffer = new ComputeBuffer(initTriangle.Count, (sizeof(int) * 3));
        triangleBuffer.SetData(initTriangle.ToArray());

        triangleIndicesBuffer = new ComputeBuffer(initTrianglePtr.Count, sizeof(int));
        triangleIndicesBuffer.SetData(initTrianglePtr.ToArray());

        Vector3[] _floorVertices = floor.GetComponent<MeshFilter>().mesh.vertices;
        List<Vector3> floorVertices = new List<Vector3>();

        for (int i = 0; i < _floorVertices.Length; i++)
        {
            floorVertices.Add(floor.transform.TransformPoint(_floorVertices[i]));
        }

        floorBBBuffer = new ComputeBuffer(1, sizeof(float) * 6);
        floorPositionsBuffer = new ComputeBuffer(_floorVertices.Length, sizeof(float) * 3);
        floorPositionsBuffer.SetData(floorVertices);

        bbMinMaxBuffer = new ComputeBuffer(numberOfObjects, sizeof(float) * 6);
        objectIndexBuffer = new ComputeBuffer(numberOfObjects, sizeof(int) * 2); 
        objectIndexBuffer.SetData(indicies);
        floorCollisionResultBuffer = new ComputeBuffer(numberOfObjects, sizeof(int));
        bbOctreeBuffer = new ComputeBuffer(numberOfObjects * octree_size, sizeof(float) * 9);
        pairIndexLv2Buffer = new ComputeBuffer(pairIndexL2.Count, sizeof(int) * 2);
        pairIndexLv2Buffer.SetData(pairIndexL2);

        collisionResults.Initialize();
        collisionResultsBuffer = new ComputeBuffer(numberOfObjects * octree_size, sizeof(int) );
        collisionResultsBuffer.SetData(collisionResults);
    }

     private void SetupKernel()
    {
        //for rendering
        computeVerticesNormal = computeShaderObj.FindKernel("computeVerticesNormal");
        updatePosKernel = computeShaderObj.FindKernel("UpdatePosKernel");
        findFloorMinMaxKernel = computeShaderObj.FindKernel("FindFloorMinMax");
        findBBMinMaxKernel = computeShader.FindKernel("FindBBMinMax");
        updateReverseVelocityKernel = computeShaderObj.FindKernel("UpdateReverseVelocity");
        implementOctreeKernel = computeShader.FindKernel("ImplementOctree");
        checkCollisionOctreeKernel = computeShaderObj.FindKernel("CheckCollisionOctree");
        removeCollisionResultsKernel = computeShaderObj.FindKernel("RemoveCollisionResults");

    }

     private void SetupComputeShader()
    {
        //send uniform data for kernels in compute shader
        computeShaderObj.SetFloat("dt", dt);
        computeShaderObj.SetBool("debug", debugData);
        computeShaderObj.SetFloat("invMass", invMass);
        computeShaderObj.SetInt("triCount", triCount);
        computeShaderObj.SetInt("nodeCount", nodeCount);
        computeShaderObj.SetInt("numberObj", numberOfObjects);

        //computeVerticesNormal
        computeShaderObj.SetBuffer(computeVerticesNormal, "Positions", positionsBuffer);
        computeShaderObj.SetBuffer(computeVerticesNormal, "Triangles", triangleBuffer);
        computeShaderObj.SetBuffer(computeVerticesNormal, "TrianglePtr", triangleIndicesBuffer);
        computeShaderObj.SetBuffer(computeVerticesNormal, "vertsBuff", vertsBuff); //passing to rendering
        
        // UpdatePosKernel
        computeShaderObj.SetBuffer(updatePosKernel, "Positions", positionsBuffer);
        computeShaderObj.SetBuffer(updatePosKernel, "Velocities", velocitiesBuffer);
        computeShaderObj.SetBuffer(updatePosKernel, "vertsBuff", vertsBuff); //passing to rendering

        // FindFloorMinMax
        computeShaderObj.SetBuffer(findFloorMinMaxKernel, "floorPositions", floorPositionsBuffer);
        computeShaderObj.SetBuffer(findFloorMinMaxKernel, "floorBB", floorBBBuffer);
        computeShaderObj.Dispatch(findFloorMinMaxKernel, 1, 1, 1);

        // findBBMinMaxKernel
        computeShaderObj.SetBuffer(findBBMinMaxKernel, "bbMinMax", bbMinMaxBuffer);
        computeShaderObj.SetBuffer(findBBMinMaxKernel, "ObjectIndex", objectIndexBuffer);
        computeShaderObj.SetBuffer(findBBMinMaxKernel, "Positions", positionsBuffer);
        computeShaderObj.SetBuffer(findBBMinMaxKernel, "floorBB", floorBBBuffer);
        computeShaderObj.SetBuffer(findBBMinMaxKernel, "floorCollisionResults", floorCollisionResultBuffer);
   
        // UpdateReverseVelocity
        computeShaderObj.SetBuffer(updateReverseVelocityKernel, "Positions", positionsBuffer);
        computeShaderObj.SetBuffer(updateReverseVelocityKernel, "Velocities", velocitiesBuffer);
        computeShaderObj.SetBuffer(updateReverseVelocityKernel, "floorCollisionResults", floorCollisionResultBuffer);
        computeShaderObj.SetBuffer(updateReverseVelocityKernel, "vertsBuff", vertsBuff); //passing to rendering
        computeShaderObj.SetBuffer(updateReverseVelocityKernel, "ObjectIndex", objectIndexBuffer);

        // ImplementOctree
        computeShaderObj.SetBuffer(implementOctreeKernel, "bbMinMax", bbMinMaxBuffer);
        computeShaderObj.SetBuffer(implementOctreeKernel, "bbOctree", bbOctreeBuffer);

        // CheckCollisionOctree
        computeShaderObj.SetBuffer(checkCollisionOctreeKernel, "bbOctree", bbOctreeBuffer);
        computeShaderObj.SetBuffer(checkCollisionOctreeKernel, "pairIndexLv2", pairIndexLv2Buffer);
        computeShaderObj.SetBuffer(checkCollisionOctreeKernel, "collisionResults", collisionResultsBuffer);

        // removeCollisionResults
        computeShaderObj.SetBuffer(removeCollisionResultsKernel, "collisionResults", collisionResultsBuffer);
    }


    void Update()
    {
        dispatchComputeShader();
        renderObject();
        GetDataToCPU();
    }

    void dispatchComputeShader()
    {
        //compute normal for rendering
        computeShaderObj.Dispatch(computeVerticesNormal, (int)Mathf.Ceil(nodeCount / 1024.0f), 1, 1);

        // // compute shader
        computeShaderObj.Dispatch(updatePosKernel, Mathf.CeilToInt(nodeCount / 1024.0f), 1, 1);
        computeShaderObj.Dispatch(findBBMinMaxKernel, Mathf.CeilToInt(numberOfObjects / 1024f), 1, 1);
        computeShaderObj.Dispatch(updateReverseVelocityKernel, Mathf.CeilToInt(nodeCount / 1024f), 1, 1);
        computeShaderObj.Dispatch(implementOctreeKernel, Mathf.CeilToInt(numberOfObjects / 1024f), 1, 1);
        computeShaderObj.Dispatch(removeCollisionResultsKernel, Mathf.CeilToInt(numberOfObjects * octree_size / 1024f), 1, 1);
        computeShaderObj.Dispatch(checkCollisionOctreeKernel, Mathf.CeilToInt(pairIndexL2.Count / 1024f), 1, 1);

    }
    void renderObject()
    {
        Bounds bounds = new Bounds(Vector3.zero, Vector3.one * 10000);
        material.SetPass(0);
        Graphics.DrawProcedural(material, bounds, MeshTopology.Triangles, triArray.Length,
            1, null, null, ShadowCastingMode.On, true, gameObject.layer);

    }

    private void GetDataToCPU()
    {
        collidableIndex.Clear();

        if (debugData)
        {
            bbOctreeBuffer.GetData(bbOctree);
            collisionResultsBuffer.GetData(collisionResults);

            // for(int i = 0; i < collisionResults.Length; i++) {
            //     if(collisionResults[i] == 1) print($"res {i} {collisionResults[i]}");
            // }
        }
       
    }

     private void OnGUI()
    {
        int w = Screen.width, h = Screen.height;
        GUIStyle style = new GUIStyle();
        Rect rect = new Rect(20, 40, w, h * 2 / 100);
        style.alignment = TextAnchor.UpperLeft;
        style.fontSize = h * 2 / 50;
        style.normal.textColor = Color.yellow;


        string text = string.Format("num. Obj :: " + numberOfObjects);
        GUI.Label(rect, text, style);
    }

    void OnDrawGizmos()
    {
        if(collisionResults != null ) 
        {
            for (int i = 0; i < collisionResults.Length; i++)
            {
                if(collisionResults[i] == 1)
                {
                    if(debugLv0)
                    {
                        pairIndexL0.Clear();
                        for (int n1 = 0; n1 < numberOfObjects; n1++)
                        {
                            var l0_st_idx = calculateStartIndex(n1, 0);
                            var l0_end_idx = calculateEndIndex(n1, 0);

                            for (int n2 = 0; n2 < numberOfObjects; n2++)
                            {
                                if (n1 == n2) break;
                                
                                var j_l0_st_idx = calculateStartIndex(n2, 0);
                                var j_l0_end_idx = calculateStartIndex(n2, 0);
                    
                                for (int n = l0_st_idx; n <= l0_end_idx; n++)
                                {
                                    for (int m = j_l0_st_idx; m <= j_l0_end_idx; m++)
                                    {
                                        // Update pair data
                                        pairIndexL0.Add( new PairData
                                        {
                                            i1 = n,
                                            i2 = m
                                        });
                                    }
                                }
                            }
                        }

                        Gizmos.color = Color.red;
                        for(int p = 0; p < pairIndexL0.Count; p++) 
                        {
                            
                            if(i == pairIndexL0[p].i1)
                            {
                                Gizmos.DrawWireCube(bbOctree[i].center, bbOctree[i].max - bbOctree[i].min);
                            }    
                            if(i == pairIndexL0[p].i2)
                            {
                                Gizmos.DrawWireCube(bbOctree[i].center, bbOctree[i].max - bbOctree[i].min);
                            }    
                        }

                    }

                    if(debugLv1)
                    {
                         pairIndexL1.Clear();
                        for (int n1 = 0; n1 < numberOfObjects; n1++)
                        {
                            var l1_st_idx = calculateStartIndex(n1, 1);
                            var l1_end_idx = calculateEndIndex(n1, 1);

                            for (int n2 = 0; n2 < numberOfObjects; n2++)
                            {
                                if (n1 == n2) break;
                              
                                var j_l1_st_idx = calculateStartIndex(n2, 1);
                                var j_l1_end_idx = calculateEndIndex(n2, 1);
                        
                                for (int n = l1_st_idx; n <= l1_end_idx; n++)
                                {
                                    for (int m = j_l1_st_idx; m <= j_l1_end_idx; m++)
                                    {
                                        pairIndexL1.Add( new PairData
                                        {
                                            i1 = n,
                                            i2 = m
                                        });
                                    }
                                }
                            }
                        }

                        for(int p = 0; p < pairIndexL1.Count; p++) 
                        {
                            Gizmos.color = Color.green;
                            if(i == pairIndexL1[p].i1)
                            {
                                
                                Gizmos.DrawWireCube(bbOctree[i].center, bbOctree[i].max - bbOctree[i].min);
                            }    
                            if(i == pairIndexL1[p].i2)
                            {
                               
                                Gizmos.DrawWireCube(bbOctree[i].center, bbOctree[i].max - bbOctree[i].min);
                            }    
                        }       
                    }

                    if(debugLv2)
                    {
                        Gizmos.color = Color.blue;
                        for(int p = 0; p < pairIndexL2.Count; p++) 
                        {
                           
                            if(i == pairIndexL2[p].i1)
                            {
                                Gizmos.DrawWireCube(bbOctree[i].center, bbOctree[i].max - bbOctree[i].min);
                            }    
                            if(i == pairIndexL2[p].i2)
                            {
                                Gizmos.DrawWireCube(bbOctree[i].center, bbOctree[i].max - bbOctree[i].min);
                            }    
                        }
                    }
                    
                }
                
        
            }
        }

    }

    private void OnDestroy() 
    {
        if (enabled)
        {
            vertsBuff.Dispose();
            triBuffer.Dispose();
            positionsBuffer.Dispose();
            triangleBuffer.Dispose();
            triangleIndicesBuffer.Dispose();
            velocitiesBuffer.Dispose();
            floorBBBuffer.Dispose();
            floorPositionsBuffer.Dispose();
            bbMinMaxBuffer.Dispose();
            objectIndexBuffer.Dispose();
            floorCollisionResultBuffer.Dispose();
            bbOctreeBuffer.Dispose();
            pairIndexLv2Buffer.Dispose();
            collisionResultsBuffer.Dispose();
        }    
    }
}