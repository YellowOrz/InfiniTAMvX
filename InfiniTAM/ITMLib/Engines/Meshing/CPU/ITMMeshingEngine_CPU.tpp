// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ITMMeshingEngine_CPU.h"
#include "../Shared/ITMMeshingEngine_Shared.h"

using namespace ITMLib;

template<class TVoxel>
void ITMMeshingEngine_CPU<TVoxel, ITMVoxelBlockHash>::MeshScene(ITMMesh *mesh,
                                                                const ITMScene<TVoxel, ITMVoxelBlockHash> *scene) {
  //! 准备相关数据
  ITMMesh::Triangle *triangles = mesh->triangles->GetData(MEMORYDEVICE_CPU);  // 获取之后三角面片存储位置的指针
  const TVoxel *localVBA = scene->localVBA.GetVoxelBlocks();                  // device上的voxel block array
  const ITMHashEntry *hashTable = scene->index.GetEntries();                  // hash table

  int noTriangles = 0;                                // 当前三角面片的数量
  int noMaxTriangles = mesh->noMaxTriangles;          // 三角片面的最大数量（常量）
  int noTotalEntries = scene->index.noTotalEntries;   // 场景中entry总数
  float factor = scene->sceneParams->voxelSize;       // voxel size，单位米

  mesh->triangles->Clear();                           // 清空mesh的三角面片
  //! 遍历每一个entry 生成mesh
  for (int entryId = 0; entryId < noTotalEntries; entryId++) {  // NOTE：不直接遍历VBA是因为相邻block之间还要生成三角面片
    const ITMHashEntry &currentHashEntry = hashTable[entryId];  // 获取entry
    if (currentHashEntry.ptr < 0) continue;   // ptr<0说明没有对应的block，跳过
    Vector3i globalPos = currentHashEntry.pos.toInt() * SDF_BLOCK_SIZE;   // 计算当前block左下角voxel的voxel坐标
    // 遍历block中的每一个voxel
    for (int z = 0; z < SDF_BLOCK_SIZE; z++)
      for (int y = 0; y < SDF_BLOCK_SIZE; y++)
        for (int x = 0; x < SDF_BLOCK_SIZE; x++) {  // NOTE: 最先遍历x
          // 以当前voxel为左下角的cube，获取其cube类型（查表，记录id），以及 8个顶点
          Vector3f vertList[12];    // 一个cube中每条边上都可能有三角面片的点，所以是12个点
          int cubeIndex = buildVertList(vertList, globalPos, Vector3i(x, y, z), localVBA, hashTable);
          // TODO：优化方案，block内部就不要查hashtable，block边缘的最多查8个（可以提前记录下来）
          if (cubeIndex < 0) continue;    // <0表示当前voxel中没有三角面片
          // 记录当前voxel中的所有三角面片
          for (int i = 0; triangleTable[cubeIndex][i] != -1; i += 3) {
            triangles[noTriangles].p0 = vertList[triangleTable[cubeIndex][i]] * factor;
            triangles[noTriangles].p1 = vertList[triangleTable[cubeIndex][i + 1]] * factor;
            triangles[noTriangles].p2 = vertList[triangleTable[cubeIndex][i + 2]] * factor;
            // TODO: 添加颜色
            if (noTriangles < noMaxTriangles - 1) noTriangles++;
          }
        }
  }

  mesh->noTotalTriangles = noTriangles;
}