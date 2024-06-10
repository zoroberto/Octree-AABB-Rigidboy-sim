using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using ExporterImporter;
using Octree;
using PBD;
using UnityEngine;
using UnityEngine.Rendering;

public class CPURigid : MonoBehaviour
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
    public Shader renderingShader;
    public Color matColor;

    [HideInInspector]
    private Material material;

    [HideInInspector]
    private int nodeCount;
    private int triCount; // size of triangle

    private Vector3[] Positions;
    private Vector3[] Velocities;
    List<Triangle> triangles = new List<Triangle>();
    //for render
    private ComputeBuffer vertsBuff = null;
    private ComputeBuffer triBuffer = null;
    private Vector2Int[] indicies;

    struct vertData
    {
        public Vector3 pos;
        public Vector2 uvs;
        public Vector3 norms;
    };
    int[] triArray;
    vertData[] vDataArray;
    private BoundingBox floorBB;
    OctreeData[] cusBB;

    // octree
    private OctreeData[] cusOctree;
    private List<PairData> pairIndexL2 = new List<PairData>();
    private readonly int octree_size = 73;

     // collidable pair
    private readonly List<int> collidablePairIndexL0 = new List<int>();
    private readonly List<int> collidablePairIndexL1 = new List<int>();
    private readonly List<PairData> collidablePairIndexL2 = new List<PairData>();
    private readonly List<Vector2Int> collidableTriIndex = new List<Vector2Int>();

     [Header("Debug Mode")]
    public bool debugLv0 = true; 
    public bool debugLv1 = true; 
    public bool debugLv2 = true; 

     void Start()
    {
        
         //obj = gameObject;
        material = new Material(renderingShader); // new material for difference object
        material.color = matColor; //set color to material

        SelectModelName();
        setupMeshData();
        setupShader();
        setBuffData();
        FindFloorMinMax();
        AddOctreePairIndex();
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

    private void setupMeshData()
    {
        var number = numberOfObjects;
        //print(Application.dataPath);
        string filePath = Application.dataPath + "/TetModel/";
        LoadTetModel.LoadData(filePath + modelName, gameObject);
        List<List<string>> csvData = ExporterAndImporter.ReadCSVFile(csv_file);
        cusOctree = new OctreeData[numberOfObjects * octree_size];
        cusBB = new OctreeData[numberOfObjects ];
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

    private void setupShader()
    {
        material.SetBuffer(Shader.PropertyToID("vertsBuff"), vertsBuff);
        material.SetBuffer(Shader.PropertyToID("triBuff"), triBuffer);
    }
    private void setBuffData()
    {

        vertsBuff.SetData(vDataArray);
        triBuffer.SetData(triArray);
        //Quaternion rotate = new Quaternion(0, 0, 0, 0);
        //transform.Rotate(rotate.eulerAngles);
        //transform.
        Vector3 translation = transform.position;
        Vector3 scale = this.transform.localScale;
        Quaternion rotationeuler = transform.rotation;
        Matrix4x4 trs = Matrix4x4.TRS(translation, rotationeuler, scale);
        material.SetMatrix("TRSMatrix", trs);
        material.SetMatrix("invTRSMatrix", trs.inverse);
    }
    
    private void FindFloorMinMax()
    {
        Vector3[] vertices;

        vertices = floor.GetComponent<MeshFilter>().mesh.vertices;
        floorBB.min = floor.transform.TransformPoint(vertices[0]);
        floorBB.max = floor.transform.TransformPoint(vertices[0]);

        for (int i = 0; i < vertices.Length; i++)
        {
            Vector3 allVerts = floor.transform.TransformPoint(vertices[i]);

            floorBB.min.x = Mathf.Min(floorBB.min.x, allVerts.x);
            floorBB.min.y = Mathf.Min(floorBB.min.y, allVerts.y);
            floorBB.min.z = Mathf.Min(floorBB.min.z, allVerts.z);

            floorBB.max.x = Mathf.Max(floorBB.max.x, allVerts.x);
            floorBB.max.y = Mathf.Max(floorBB.max.y, allVerts.y);
            floorBB.max.z = Mathf.Max(floorBB.max.z, allVerts.z);
            floorBB.max.y += 0.01f;
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
        //if(debugLv2)
        {
            for(int i = 0; i < numberOfObjects; i++)
            {
                for (int j = 0; j < numberOfObjects; j++)
                {
                    var l2_st_idx = calculateStartIndex(i, 2);
                    var l2_end_idx = calculateEndIndex(i, 2);

                    if (i == j) break;

                    var j_l2_st_idx = calculateStartIndex(j, 2);
                    var j_l2_end_idx = calculateEndIndex(j, 2);

                    for (int n = l2_st_idx; n < l2_end_idx; n++)
                    {
                        for (int m = j_l2_st_idx; m < j_l2_end_idx; m++)
                        {
                            pairIndexL2.Add(new PairData
                            {
                                i1 = n,
                                i2 = m,
                            });
                        }
                    }
                }            
            }
        }
    }

   
    void Update()
    {
        UpdateNodes();
        ImplementOctree();
        //CheckCollisionWithFloor();
        UpdateReverseVelocity();
        CheckCollisionOctree();

        computeVertexNormal();
        vertsBuff.SetData(vDataArray);
        Bounds bounds = new Bounds(Vector3.zero, Vector3.one * 100);
        material.SetPass(0);
        Graphics.DrawProcedural(material, bounds, MeshTopology.Triangles, triArray.Length,
            1, null, null, ShadowCastingMode.On, true, gameObject.layer);
    }

    void computeVertexNormal()
    {
        for (int i = 0; i < triCount; i++)
        {
            //print(TriIndices[i * 3 + 0]+","+ TriIndices[i * 3 + 1] + "," +TriIndices[i * 3 + 2]);
            //
            Vector3 v1 = Positions[triArray[i * 3 + 0]];
            Vector3 v2 = Positions[triArray[i * 3 + 1]];
            Vector3 v3 = Positions[triArray[i * 3 + 2]];

            Vector3 N = Vector3.Cross(v2 - v1, v3 - v1);

            vDataArray[triArray[i * 3 + 0]].norms += N;
            vDataArray[triArray[i * 3 + 1]].norms += N;
            vDataArray[triArray[i * 3 + 2]].norms += N;
        }
        for (int i = 0; i < nodeCount; i++)
        {
            vDataArray[i].norms = vDataArray[i].norms.normalized;
        }
    }

    void UpdateNodes()
    {
        //Euler method
        for (int i = 0; i < nodeCount; i++)
        {
            Vector3 pos = Positions[i];
            Vector3 vel = Velocities[i];

            vel = vel + gravity * invMass * dt;
            pos = pos + vel * dt;

            Positions[i] = pos;
            Velocities[i] = vel;

            vDataArray[i].pos = Positions[i];
        }
    }

     private void CheckCollisionWithFloor()
    {
        for(int i = 0; i < cusBB.Length; i++) 
        {
            floorBB.collide = Intersection.AABB(cusBB[i].min, cusBB[i].max, floorBB.min, floorBB.max);
            // print($"{i} {floorBB.collide}");
        }
    }

    private void UpdateReverseVelocity() 
    {
        for (int i = 0; i < nodeCount; i++)
        {
            for(int j = 0; j < cusBB.Length; j++) {
                floorBB.collide = Intersection.AABB(cusBB[j].min, cusBB[j].max, 
                floorBB.min, floorBB.max); 
                
                var start = indicies[j].x;
                var end = indicies[j].y;
                
                if (floorBB.collide){
                    if(i>= start && i < end ) {
                        // print("index "+ i);
                        Velocities[i] *= -1;
                        Positions[i].y += .1f;
                        vDataArray[i].pos = Positions[i];
                    }
                }
            }
        }
    }

    private void ImplementOctree() 
    {
        for(int i = 0; i < numberOfObjects; i++) {
            var _start = indicies[i].x;
            var _end = indicies[i].y;

            Vector3 minPos = Positions[_start]; // 0 26
            Vector3 maxPos = Positions[_start]; // 0 26

            for(var s = _start; s < _end; s++) {
                Vector3 vertex = Positions[s];

                minPos.x = Mathf.Min(minPos.x, vertex.x);
                minPos.y = Mathf.Min(minPos.y, vertex.y);
                minPos.z = Mathf.Min(minPos.z, vertex.z);

                maxPos.x = Mathf.Max(maxPos.x, vertex.x);
                maxPos.y = Mathf.Max(maxPos.y, vertex.y);
                maxPos.z = Mathf.Max(maxPos.z, vertex.z);

                cusBB[i].min = minPos;
                cusBB[i].max = maxPos;

                 // Lv0, Initialize Lv0
                cusOctree[i * octree_size].center = (cusBB[i].max + cusBB[i].min) / 2; // center Lv0
                cusOctree[i * octree_size].min = cusBB[i].min; // min value Lv0
                cusOctree[i * octree_size].max = cusBB[i].max; // max value Lv0

                Vector3 centerOct = cusOctree[i * octree_size].center;
                Vector3 sizeOct = cusOctree[i * octree_size].max - cusOctree[i * octree_size].min;

                // Lv2, Split to 8 children [0-7] 
                cusOctree[i * octree_size + 1].center.x = centerOct.x - (sizeOct.x / 4);
                cusOctree[i * octree_size + 1].center.y = centerOct.y + (sizeOct.y / 4);
                cusOctree[i * octree_size + 1].center.z = centerOct.z - (sizeOct.z / 4);

                cusOctree[i * octree_size + 2].center.x = centerOct.x + (sizeOct.x / 4);
                cusOctree[i * octree_size + 2].center.y = centerOct.y + (sizeOct.y / 4);
                cusOctree[i * octree_size + 2].center.z = centerOct.z - (sizeOct.z / 4);

                cusOctree[i * octree_size + 3].center.x = centerOct.x - (sizeOct.x / 4);
                cusOctree[i * octree_size + 3].center.y = centerOct.y - (sizeOct.y / 4);
                cusOctree[i * octree_size + 3].center.z = centerOct.z - (sizeOct.z / 4);

                cusOctree[i * octree_size + 4].center.x = centerOct.x + (sizeOct.x / 4);
                cusOctree[i * octree_size + 4].center.y = centerOct.y - (sizeOct.y / 4);
                cusOctree[i * octree_size + 4].center.z = centerOct.z - (sizeOct.z / 4);

                cusOctree[i * octree_size + 5].center.x = centerOct.x - (sizeOct.x / 4);
                cusOctree[i * octree_size + 5].center.y = centerOct.y + (sizeOct.y / 4);
                cusOctree[i * octree_size + 5].center.z = centerOct.z + (sizeOct.z / 4);

                cusOctree[i * octree_size + 6].center.x = centerOct.x + (sizeOct.x / 4);
                cusOctree[i * octree_size + 6].center.y = centerOct.y + (sizeOct.y / 4);
                cusOctree[i * octree_size + 6].center.z = centerOct.z + (sizeOct.z / 4);

                cusOctree[i * octree_size + 7].center.x = centerOct.x - (sizeOct.x / 4);
                cusOctree[i * octree_size + 7].center.y = centerOct.y - (sizeOct.y / 4);
                cusOctree[i * octree_size + 7].center.z = centerOct.z + (sizeOct.z / 4);

                cusOctree[i * octree_size + 8].center.x = centerOct.x + (sizeOct.x / 4);
                cusOctree[i * octree_size + 8].center.y = centerOct.y - (sizeOct.y / 4);
                cusOctree[i * octree_size + 8].center.z = centerOct.z + (sizeOct.z / 4);

                
                for (int j = 1; j <= 8; j++)
                {
                    OctreeData oct = cusOctree[i * octree_size + j];
                    cusOctree[i * octree_size + j].min = oct.Minimum(oct.center, sizeOct / 4);
                    cusOctree[i * octree_size + j].max = oct.Maximum(oct.center, sizeOct / 4);

                    // Lv2, Split to 64 children

                    cusOctree[i * octree_size + j * 8 + 1].center.x = cusOctree[i * octree_size + j].center.x - (sizeOct.x / 8);
                    cusOctree[i * octree_size + j * 8 + 1].center.y = cusOctree[i * octree_size + j].center.y + (sizeOct.y / 8);
                    cusOctree[i * octree_size + j * 8 + 1].center.z = cusOctree[i * octree_size + j].center.z - (sizeOct.z / 8);

                    cusOctree[i * octree_size + j * 8 + 2].center.x = cusOctree[i * octree_size + j].center.x + (sizeOct.x / 8);
                    cusOctree[i * octree_size + j * 8 + 2].center.y = cusOctree[i * octree_size + j].center.y + (sizeOct.y / 8);
                    cusOctree[i * octree_size + j * 8 + 2].center.z = cusOctree[i * octree_size + j].center.z - (sizeOct.z / 8);

                    cusOctree[i * octree_size + j * 8 + 3].center.x = cusOctree[i * octree_size + j].center.x - (sizeOct.x / 8);
                    cusOctree[i * octree_size + j * 8 + 3].center.y = cusOctree[i * octree_size + j].center.y - (sizeOct.y / 8);
                    cusOctree[i * octree_size + j * 8 + 3].center.z = cusOctree[i * octree_size + j].center.z - (sizeOct.z / 8);

                    cusOctree[i * octree_size + j * 8 + 4].center.x = cusOctree[i * octree_size + j].center.x + (sizeOct.x / 8);
                    cusOctree[i * octree_size + j * 8 + 4].center.y = cusOctree[i * octree_size + j].center.y - (sizeOct.y / 8);
                    cusOctree[i * octree_size + j * 8 + 4].center.z = cusOctree[i * octree_size + j].center.z - (sizeOct.z / 8);

                    cusOctree[i * octree_size + j * 8 + 5].center.x = cusOctree[i * octree_size + j].center.x - (sizeOct.x / 8);
                    cusOctree[i * octree_size + j * 8 + 5].center.y = cusOctree[i * octree_size + j].center.y + (sizeOct.y / 8);
                    cusOctree[i * octree_size + j * 8 + 5].center.z = cusOctree[i * octree_size + j].center.z + (sizeOct.z / 8);

                    cusOctree[i * octree_size + j * 8 + 6].center.x = cusOctree[i * octree_size + j].center.x + (sizeOct.x / 8);
                    cusOctree[i * octree_size + j * 8 + 6].center.y = cusOctree[i * octree_size + j].center.y + (sizeOct.y / 8);
                    cusOctree[i * octree_size + j * 8 + 6].center.z = cusOctree[i * octree_size + j].center.z + (sizeOct.z / 8);

                    cusOctree[i * octree_size + j * 8 + 7].center.x = cusOctree[i * octree_size + j].center.x - (sizeOct.x / 8);
                    cusOctree[i * octree_size + j * 8 + 7].center.y = cusOctree[i * octree_size + j].center.y - (sizeOct.y / 8);
                    cusOctree[i * octree_size + j * 8 + 7].center.z = cusOctree[i * octree_size + j].center.z + (sizeOct.z / 8);

                    cusOctree[i * octree_size + j * 8 + 8].center.x = cusOctree[i * octree_size + j].center.x + (sizeOct.x / 8);
                    cusOctree[i * octree_size + j * 8 + 8].center.y = cusOctree[i * octree_size + j].center.y - (sizeOct.y / 8);
                    cusOctree[i * octree_size + j * 8 + 8].center.z = cusOctree[i * octree_size + j].center.z + (sizeOct.z / 8);

                    for (int k = 1; k <= 8; k++)
                    {
                        cusOctree[i * octree_size + j * 8 + k].min = oct.Minimum(cusOctree[i * octree_size + j * 8 + k].center, sizeOct / 8);
                        cusOctree[i * octree_size + j * 8 + k].max = oct.Maximum(cusOctree[i * octree_size + j * 8 + k].center, sizeOct / 8);
                    }
                }
            }
        }
    }

    
     private int calcIndexObjectLevel0(int object_index)
    {
        int level = (int)Mathf.Floor(object_index / octree_size);

        //if (level == 1) return level * 73;
        if (level != 0) return level * octree_size;
        return (int)Mathf.Floor(object_index / octree_size);
    }

     private int calcIndexObjectLevel1(int object_index )
    {
        int level = (int)Mathf.Floor(object_index / octree_size);

        if (level == 1) return (int)Mathf.Floor((object_index - (1 + level)) / 8) + level * 64;
        if (level == 2) return (int)Mathf.Floor((object_index - (1 + level)) / 8) + level * 64;
        return (int)Mathf.Floor((object_index - 1) / 8) + level * 64 ;
    }
    private void CheckCollisionOctree()
    {

        collidablePairIndexL0.Clear();
        collidablePairIndexL1.Clear();
        collidablePairIndexL2.Clear();
            
        for (int i = 0; i < pairIndexL2.Count; i++)
        {
            int l0_index_obj1 = calcIndexObjectLevel0(pairIndexL2[i].i1);
            int l0_index_obj2 = calcIndexObjectLevel0(pairIndexL2[i].i2);

            int l1_index_obj1 = calcIndexObjectLevel1(pairIndexL2[i].i1);
            int l1_index_obj2 = calcIndexObjectLevel1(pairIndexL2[i].i2);

            // if (Intersection.AABB(cusOctree[l0_index_obj1].min, cusOctree[l0_index_obj1].max,
            //     cusOctree[l0_index_obj2].min, cusOctree[l0_index_obj2].max))
            // {
            //     if (debugLv0)
            //     {
            //         // print($"index {l0_index_obj1} {l0_index_obj2}");
            //         if (!collidablePairIndexL0.Contains(l0_index_obj1))
            //             collidablePairIndexL0.Add(l0_index_obj1);

            //         if (!collidablePairIndexL0.Contains(l0_index_obj2))
            //             collidablePairIndexL0.Add(l0_index_obj2);
                        
            //     }
            //     if (Intersection.AABB(cusOctree[l1_index_obj1].min, cusOctree[l1_index_obj1].max,
            //         cusOctree[l1_index_obj2].min, cusOctree[l1_index_obj2].max))
            //     {
            //         if (debugLv1)
            //         {
            //             if (!collidablePairIndexL1.Contains(l1_index_obj1))
            //                 collidablePairIndexL1.Add(l1_index_obj1);

            //             if (!collidablePairIndexL1.Contains(l1_index_obj2))
            //                 collidablePairIndexL1.Add(l1_index_obj2);
            //         }

            //         if (Intersection.AABB(cusOctree[pairIndexL2[i].i1].min, cusOctree[pairIndexL2[i].i1].max,
            //             cusOctree[pairIndexL2[i].i2].min, cusOctree[pairIndexL2[i].i2].max))
            //         {
            //             if (debugLv2)
            //             {
            //                 if (!collidablePairIndexL2.Any(c => c.Equals(pairIndexL2[i])))
            //                     collidablePairIndexL2.Add(pairIndexL2[i]);
            //             }
            //         }
            //     }
            // }

            if(!Intersection.AABB(cusOctree[l0_index_obj1].min, cusOctree[l0_index_obj1].max,
                cusOctree[l0_index_obj2].min, cusOctree[l0_index_obj2].max))
            {
                continue;
            }
            else
            {
                if (debugLv0)
                {
                    // print($"index {l0_index_obj1} {l0_index_obj2}");
                    if (!collidablePairIndexL0.Contains(l0_index_obj1))
                        collidablePairIndexL0.Add(l0_index_obj1);

                    if (!collidablePairIndexL0.Contains(l0_index_obj2))
                        collidablePairIndexL0.Add(l0_index_obj2);
                }
            }

            if (!Intersection.AABB(cusOctree[l1_index_obj1].min, cusOctree[l1_index_obj1].max,
            cusOctree[l1_index_obj2].min, cusOctree[l1_index_obj2].max))
            {
                continue;
            }
            else
            {
                if (debugLv1)
                {
                    if (!collidablePairIndexL1.Contains(l1_index_obj1))
                        collidablePairIndexL1.Add(l1_index_obj1);

                    if (!collidablePairIndexL1.Contains(l1_index_obj2))
                        collidablePairIndexL1.Add(l1_index_obj2);
                }
            }

            if (!Intersection.AABB(cusOctree[pairIndexL2[i].i1].min, cusOctree[pairIndexL2[i].i1].max,
            cusOctree[pairIndexL2[i].i2].min, cusOctree[pairIndexL2[i].i2].max))
            {
                if (debugLv2)
                {
                    if (!collidablePairIndexL2.Any(c => c.Equals(pairIndexL2[i])))
                        collidablePairIndexL2.Add(pairIndexL2[i]);
                }
            }
        }
    }

    private void OnDrawGizmos()
    {
        // if (cusOctree != null )
        // {
           
        //     for (int i = 0; i < cusOctree.Length; i++)
        //     {
        //         if (debugLv0)
        //         {
        //             Gizmos.color = Color.red;
        //             for (int j = 0; j < collidablePairIndexL0.Count; j++)
        //             {
        //                 if (i == collidablePairIndexL0[j])
        //                 {
        //                     Vector3 size = cusOctree[collidablePairIndexL0[j]].max - cusOctree[collidablePairIndexL0[j]].min;
        //                     Gizmos.DrawWireCube(cusOctree[collidablePairIndexL0[j]].center, size);
        //                 }
        //             }
        //         }

        //         if (debugLv1)
        //         {
        //             Gizmos.color = Color.green;
        //             for (int j = 0; j < collidablePairIndexL1.Count; j++)
        //             {
        //                 if (i == collidablePairIndexL1[j])
        //                 {
        //                     Vector3 size = cusOctree[collidablePairIndexL1[j]].max - cusOctree[collidablePairIndexL1[j]].min;
        //                     Gizmos.DrawWireCube(cusOctree[collidablePairIndexL1[j]].center, size);
        //                 }
                        
        //             }
        //         }

        //         if (debugLv2)
        //         {
        //             Gizmos.color = Color.blue;
        //             for (int j = 0; j < collidablePairIndexL2.Count; j++)
        //             {
                       
        //                 if (i == collidablePairIndexL2[j].i1)
        //                 {
        //                     Vector3 size = cusOctree[collidablePairIndexL2[j].i1].max - cusOctree[collidablePairIndexL2[j].i1].min;
        //                     Gizmos.DrawWireCube(cusOctree[collidablePairIndexL2[j].i1].center, size);
        //                 }
        //                 if (i == collidablePairIndexL2[j].i2)
        //                 {
        //                     Vector3 size = cusOctree[collidablePairIndexL2[j].i2].max - cusOctree[collidablePairIndexL2[j].i2].min;
        //                     Gizmos.DrawWireCube(cusOctree[collidablePairIndexL2[j].i2].center, size);
        //                 }
        //             }
        //         }
        //     }
        // }
    }

    private void OnDestroy()
    {
        if (enabled)
        {
            vertsBuff.Dispose();
            triBuffer.Dispose();
        }
    }
}
