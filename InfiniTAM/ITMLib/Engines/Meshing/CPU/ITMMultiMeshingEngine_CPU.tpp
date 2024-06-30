// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ITMMultiMeshingEngine_CPU.h"

#include "../Shared/ITMMultiMeshingEngine_Shared.h"

using namespace ITMLib;

template<class TVoxel>
inline void ITMMultiMeshingEngine_CPU<TVoxel, ITMVoxelBlockHash>::MeshScene(ITMMesh *mesh,
                                                                            const MultiSceneManager &sceneManager) {
  int numLocalMaps = (int) sceneManager.numLocalMaps();                                   // 子图的总数
  if (numLocalMaps > MAX_NUM_LOCALMAPS) numLocalMaps = MAX_NUM_LOCALMAPS;                 // 限制能处理的子图数量
  //! 遍历每个子图，获取相关信息。
  MultiIndexData hashTables;
  MultiVoxelData localVBAs;
  const ITMSceneParams &sceneParams = *(sceneManager.getLocalMap(0)->scene->sceneParams); // 三维场景的参数，所有子图都一样
  hashTables.numLocalMaps = numLocalMaps;
  for (int localMapId = 0; localMapId < numLocalMaps; ++localMapId) {
    // 获取每个子图在voxel坐标系下的世界到子图的位姿
    hashTables.poses_vs[localMapId] = sceneManager.getEstimatedGlobalPose(localMapId).GetM();
    hashTables.poses_vs[localMapId].m30 /= sceneParams.voxelSize;
    hashTables.poses_vs[localMapId].m31 /= sceneParams.voxelSize;
    hashTables.poses_vs[localMapId].m32 /= sceneParams.voxelSize;
    // TODO: 应该是真实坐标系下子图到世界的位姿，不用除voxelSize
    hashTables.posesInv[localMapId] = sceneManager.getEstimatedGlobalPose(localMapId).GetInvM();
    hashTables.posesInv[localMapId].m30 /= sceneParams.voxelSize;
    hashTables.posesInv[localMapId].m31 /= sceneParams.voxelSize;
    hashTables.posesInv[localMapId].m32 /= sceneParams.voxelSize;
    // 获取每个子图的hash table的指针
    hashTables.index[localMapId] = sceneManager.getLocalMap(localMapId)->scene->index.getIndexData();
    // 获取每个子图的voxel block array的指针
    localVBAs.voxels[localMapId] = sceneManager.getLocalMap(localMapId)->scene->localVBA.GetVoxelBlocks();
  }

  ITMMesh::Triangle *triangles = mesh->triangles->GetData(MEMORYDEVICE_CPU);    // 获取之后三角面片存储位置的指针
  mesh->triangles->Clear();

  int noTriangles = 0, noMaxTriangles = mesh->noMaxTriangles;                   // 三角片面的最大数量（常量）
  int noTotalEntriesPerLocalMap = ITMVoxelBlockHash::noTotalEntries;            // 场景中entry总数
  float factor = sceneParams.voxelSize;                                         // voxel size，单位米

  //! 遍历每个子图
  // NOTE: 过于冗余，因为同一个voxel在多个子图中都有数据。very dumb rendering -- likely to generate lots of duplicates
  for (int localMapId = 0; localMapId < numLocalMaps; ++localMapId) {
    
    ITMHashEntry *hashTable = hashTables.index[localMapId];
    //! 遍历当前子图中的每个entry
    for (int entryId = 0; entryId < noTotalEntriesPerLocalMap; entryId++) {
      // 找到有效的entry（即有对应的voxel block）
      const ITMHashEntry &currentHashEntry = hashTable[entryId];
      if (currentHashEntry.ptr < 0) continue;
      // 获取当前entry对应的voxel block的block 坐标
      Vector3i globalPos = currentHashEntry.pos.toInt() * SDF_BLOCK_SIZE;
      // 遍历voxel block中的每个voxel，抽取mesh
      for (int z = 0; z < SDF_BLOCK_SIZE; z++)
        for (int y = 0; y < SDF_BLOCK_SIZE; y++)
          for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
            // 以当前voxel为左下角的cube，获取其cube类型（查表，记录id），以及 8个顶点
            Vector3f vertList[12];
            int cubeIndex =
                buildVertListMulti(vertList, globalPos, Vector3i(x, y, z), &localVBAs, &hashTables, localMapId);

            if (cubeIndex < 0) continue;  // <0表示当前voxel中没有三角面片

            for (int i = 0; triangleTable[cubeIndex][i] != -1; i += 3) {
              triangles[noTriangles].p0 = vertList[triangleTable[cubeIndex][i]] * factor;
              triangles[noTriangles].p1 = vertList[triangleTable[cubeIndex][i + 1]] * factor;
              triangles[noTriangles].p2 = vertList[triangleTable[cubeIndex][i + 2]] * factor;
              // TODO: 添加颜色
              if (noTriangles < noMaxTriangles - 1) noTriangles++;
            }
          }
    }
  }

  mesh->noTotalTriangles = noTriangles;
}