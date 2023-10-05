// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Scene/ITMVoxelBlockHash.h"
#include "../../../ORUtils/Image.h"

#include <stdlib.h>

namespace ITMLib {
class ITMMesh {
 public:
  struct Triangle { Vector3f p0, p1, p2; };     // 三角面片 结构体

  MemoryDeviceType memoryType;

  uint noTotalTriangles;                        // mesh中三角面片数量
  static const uint noMaxTriangles_default = SDF_LOCAL_BLOCK_NUM * 32 * 16;
  uint noMaxTriangles;                          // mesh中三角面片 最大数量

  ORUtils::MemoryBlock<Triangle> *triangles;    // mesh中三角面片数据

  explicit ITMMesh(MemoryDeviceType memoryType, uint maxTriangles = noMaxTriangles_default) {
    this->memoryType = memoryType;
    this->noTotalTriangles = 0;
    this->noMaxTriangles = maxTriangles;

    triangles = new ORUtils::MemoryBlock<Triangle>(noMaxTriangles, memoryType);
  }
  /**
   * @brief 将mesh保存成obj文件
   * @param fileName 
   */
  void WriteOBJ(const char *fileName) {
    //! 准备
    ORUtils::MemoryBlock<Triangle> *cpu_triangles;
    bool shoulDelete = false;
    if (memoryType == MEMORYDEVICE_CUDA) {  // GPU上的要先down下来
      cpu_triangles = new ORUtils::MemoryBlock<Triangle>(noMaxTriangles, MEMORYDEVICE_CPU);
      cpu_triangles->SetFrom(triangles, ORUtils::MemoryBlock<Triangle>::CUDA_TO_CPU);
      shoulDelete = true;
    } else cpu_triangles = triangles;

    Triangle *triangleArray = cpu_triangles->GetData(MEMORYDEVICE_CPU);
    //! 保存
    FILE *f = fopen(fileName, "w+");  // TODO:这里应该弄成wb+
    if (f != NULL) {
      for (uint i = 0; i < noTotalTriangles; i++) { // 顶点信息
        fprintf(f, "v %f %f %f\n", triangleArray[i].p0.x, triangleArray[i].p0.y, triangleArray[i].p0.z);
        fprintf(f, "v %f %f %f\n", triangleArray[i].p1.x, triangleArray[i].p1.y, triangleArray[i].p1.z);
        fprintf(f, "v %f %f %f\n", triangleArray[i].p2.x, triangleArray[i].p2.y, triangleArray[i].p2.z);
      }

      for (uint i = 0; i < noTotalTriangles; i++)   // 面片信息
        fprintf(f,
                "f %d %d %d\n",
                i * 3 + 2 + 1,
                i * 3 + 1 + 1,
                i * 3 + 0 + 1);
      fclose(f);
    }

    if (shoulDelete) delete cpu_triangles;
  }
  /**
   * @brief 将mesh保存成stl文件
   * @param fileName 
   */
  void WriteSTL(const char *fileName) {
    //! 准备
    ORUtils::MemoryBlock<Triangle> *cpu_triangles;
    bool shoulDelete = false;
    if (memoryType == MEMORYDEVICE_CUDA) {  // GPU上的要先down下来
      cpu_triangles = new ORUtils::MemoryBlock<Triangle>(noMaxTriangles, MEMORYDEVICE_CPU);
      cpu_triangles->SetFrom(triangles, ORUtils::MemoryBlock<Triangle>::CUDA_TO_CPU);
      shoulDelete = true;
    } else cpu_triangles = triangles;

    Triangle *triangleArray = cpu_triangles->GetData(MEMORYDEVICE_CPU);
    //! 保存
    FILE *f = fopen(fileName, "wb+");
    if (f != NULL) {
      for (int i = 0; i < 80; i++) fwrite(" ", sizeof(char), 1, f);

      fwrite(&noTotalTriangles, sizeof(int), 1, f);

      float zero = 0.0f;
      short attribute = 0;
      for (uint i = 0; i < noTotalTriangles; i++) {
        fwrite(&zero, sizeof(float), 1, f);   // TODO: RGB???
        fwrite(&zero, sizeof(float), 1, f);
        fwrite(&zero, sizeof(float), 1, f);

        fwrite(&triangleArray[i].p2.x, sizeof(float), 1, f);
        fwrite(&triangleArray[i].p2.y, sizeof(float), 1, f);
        fwrite(&triangleArray[i].p2.z, sizeof(float), 1, f);

        fwrite(&triangleArray[i].p1.x, sizeof(float), 1, f);
        fwrite(&triangleArray[i].p1.y, sizeof(float), 1, f);
        fwrite(&triangleArray[i].p1.z, sizeof(float), 1, f);

        fwrite(&triangleArray[i].p0.x, sizeof(float), 1, f);
        fwrite(&triangleArray[i].p0.y, sizeof(float), 1, f);
        fwrite(&triangleArray[i].p0.z, sizeof(float), 1, f);

        fwrite(&attribute, sizeof(short), 1, f);

        //fprintf(f, "v %f %f %f\n", triangleArray[i].p0.x, triangleArray[i].p0.y, triangleArray[i].p0.z);
        //fprintf(f, "v %f %f %f\n", triangleArray[i].p1.x, triangleArray[i].p1.y, triangleArray[i].p1.z);
        //fprintf(f, "v %f %f %f\n", triangleArray[i].p2.x, triangleArray[i].p2.y, triangleArray[i].p2.z);
      }

      //for (uint i = 0; i<noTotalTriangles; i++) fprintf(f, "f %d %d %d\n", i * 3 + 2 + 1, i * 3 + 1 + 1, i * 3 + 0 + 1);
      fclose(f);
    }

    if (shoulDelete) delete cpu_triangles;
  }

  ~ITMMesh() {
    delete triangles;
  }

  // Suppress the default copy constructor and assignment operator
  ITMMesh(const ITMMesh &);
  ITMMesh &operator=(const ITMMesh &);
};
}
