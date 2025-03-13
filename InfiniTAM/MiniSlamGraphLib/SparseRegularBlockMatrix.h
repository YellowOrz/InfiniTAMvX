// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include <map>

#include "SparseBlockMatrix.h"

namespace MiniSlamGraph {

/**
 * @brief                 稀疏、等大的块矩阵
 * @details               继承自SparseBlockMatrix，使用非类型模板参数。“等大”的意思是每一个块矩阵的大小都一样
 * @tparam BlockSizeRows  块的高度（行数）
 * @tparam BlockSizeCols  块的宽度（列数）
 */
template <int BlockSizeRows, int BlockSizeCols>
class SparseRegularBlockMatrix : public SparseBlockMatrix {
 public:
  static const int bsRows = BlockSizeRows;  // 块的行数 //? 相当于只能有一种block size的SparseRegularBlockMatrix？还是使用不同的参数生成具体的类时，都不一样？
  static const int bsCols = BlockSizeCols;  // 块的列数

  /** 块的行列位置 */
  struct BlockIndex {
    int block_r, block_c;  // 块的行号和列号
    /** 默认构造函数 */
    BlockIndex(void) {}
    /** 构造函数，初始化块的坐标 */
    BlockIndex(int r, int c) {
      block_r = r;
      block_c = c;
    }

    /**
     * @brief         比较块的位置（按行优先排序），用于std::map的键排序。
     * @param[in] b   要比较的另一个BlockIndex对象
     * @return bool   =true，表示当前块比b更靠前
     */
    bool operator<(const BlockIndex &b) const {
      if (this->block_r < b.block_r) return true;
      if (this->block_r > b.block_r) return false;
      if (this->block_c < b.block_c) return true;
      return false;
    }
  };

  /**
   * @brief   存储块中的数据
   * @details 数据以一维数组形式存储，大小为BlockSizeRows × BlockSizeCols。
   */
  struct BlockData {
    double data[BlockSizeRows * BlockSizeCols];  // 块元素数据数组
    // BlockData(void) { for (int i = 0; i < BlockSizeRows*BlockSizeCols; ++i) data[i] = 0.0f; }
    /** 用一维坐标访问数据 */
    double &operator[](int idx) { return data[idx]; }
    const double &operator[](int idx) const { return data[idx]; }
  };
  typedef std::map<BlockIndex, BlockData> MatrixData;  // 块存储容器类型定义

  /**
   * @brief 向矩阵中添加或累加一个块数据。
   * @param[in] row   块左上角元素的行号
   * @param[in] col   块左上角元素的列号
   * @param[in] nr    块的实际行数（必须等于BlockSizeRows）
   * @param[in] nc    块的实际列数（必须等于BlockSizeCols）
   * @param[in] data  块的原始数据指针
   * @return bool     操作是否成功。若块尺寸不符或位置不正确则返回false。
   * @note 当块已存在时执行累加操作，否则插入新块
   */
  bool addBlock(int row, int col, int nr, int nc, double *data) {
    int block_row = row / BlockSizeRows;
    int block_col = col / BlockSizeCols;
    // TODO: handle ERROR
    if ((nr != BlockSizeRows) || (nc != BlockSizeCols)) return false;                            // 保证尺寸一致
    if ((row != block_row * BlockSizeRows) || (col != block_col * BlockSizeCols)) return false;  // 保证坐标对齐

    typename MatrixData::iterator it = mData.find(BlockIndex(block_row, block_col));

    if (it != mData.end()) {  // 已经存在，累加
      for (int i = 0; i < BlockSizeRows * BlockSizeCols; ++i) it->second[i] += data[i];
    } else {                  // 否则新增
      BlockData d;
      for (int i = 0; i < BlockSizeRows * BlockSizeCols; ++i) d[i] = data[i];
      mData.insert(std::make_pair(BlockIndex(block_row, block_col), d));
    }
    return true;
  }

  /**
   * @brief                 获取矩阵统计信息
   * @param[out] numRows    矩阵总行数
   * @param[out] numCols    矩阵总列数
   * @param[out] numEntries 元素总数
   */
  void getStats(int &numRows, int &numCols, int &numEntries) const {
    numRows = -1;
    numCols = -1;
    numEntries = 0;

    typename MatrixData::const_iterator it = mData.begin();
    for (; it != mData.end(); ++it) {
      int blockPos_r = it->first.block_r;
      int blockPos_c = it->first.block_c;
      if (blockPos_r > numRows) numRows = blockPos_r;
      if (blockPos_c > numCols) numCols = blockPos_c;
      numEntries += BlockSizeRows * BlockSizeCols;
    }
    numRows = (numRows + 1) * BlockSizeRows;
    numCols = (numCols + 1) * BlockSizeCols;
  }

  /**
   * @brief                 将矩阵转换为三元组格式
   * @param[out] rowIndices 元素的行坐标
   * @param[out] colIndices 元素的列坐标
   * @param[out] data       元素的值
   * @return int            实际填充的元素数量
   * @note                  返回值与numEntries一致
   */
  int toTriplets(int *rowIndices, int *colIndices, double *data) const {
    // TODO: untested. should be fine!
    int numEntries = 0;
    typename MatrixData::const_iterator it = mData.begin();
    for (; it != mData.end(); ++it) {
      int blockPos_r = it->first.block_r * BlockSizeRows;
      int blockPos_c = it->first.block_c * BlockSizeCols;
      for (int r = 0; r < BlockSizeRows; ++r)
        for (int c = 0; c < BlockSizeCols; ++c) {
          rowIndices[numEntries] = blockPos_r + r;
          colIndices[numEntries] = blockPos_c + c;
          data[numEntries] = it->second[r * BlockSizeRows + c];
          ++numEntries;
        }
    }
    return numEntries;
  }

  /**
   * @brief                   将矩阵转换为压缩列存储格式（CSC）
   * @param[out] rowIndices   元素行索引数组
   * @param[out] colPointers  列指针数组
   * @param[out] data         元素值数组
   * @details                 需确保rowIndices、colPointers和data已正确好足够的内存
   */
  void toCompressedColumns(int *rowIndices, int *colPointers, double *data) const {
    typename MatrixData::const_iterator it = mData.begin();
    //! 统计每列的块数量
    std::vector<int> entriesPerColumn_blockwise;
    for (; it != mData.end(); ++it) {
      int blockPos_c = it->first.block_c;
      if (entriesPerColumn_blockwise.size() < (size_t)(blockPos_c + 1))
        entriesPerColumn_blockwise.resize(blockPos_c + 1, 0);
      entriesPerColumn_blockwise[blockPos_c] += 1;
    }
    //! 计算每列的指针：即计算矩阵中每一列的元素在data中的哪一段
    int columnOffset = 0;
    for (size_t blockIdx_c = 0; blockIdx_c < entriesPerColumn_blockwise.size(); ++blockIdx_c) {
      for (int i = 0; i < BlockSizeCols; ++i) {
        // fprintf(stderr, "in column %i: %i entries starting at offset %i\n", blockIdx_c*BlockSizeCols+i,
        //    entriesPerColumn_blockwise[blockIdx_c] * BlockSizeRows, columnOffset);
        colPointers[blockIdx_c * BlockSizeCols + i] = columnOffset;
        columnOffset += entriesPerColumn_blockwise[blockIdx_c] * BlockSizeRows;
      }
    }
    colPointers[entriesPerColumn_blockwise.size() * BlockSizeCols] = columnOffset;
    //! 填充数据
    int numEntries = 0;
    for (size_t i = 0; i < entriesPerColumn_blockwise.size(); ++i) entriesPerColumn_blockwise[i] = 0;
    for (it = mData.begin(); it != mData.end(); ++it) {
      int blockIdx_c = it->first.block_c;
      int blockPos_r = it->first.block_r * BlockSizeRows;
      int blockPos_c = blockIdx_c * BlockSizeCols;
      // fprintf(stderr, "placing block at %i %i\n", blockPos_r, blockPos_c);
      for (int r = 0; r < BlockSizeRows; ++r)
        for (int c = 0; c < BlockSizeCols; ++c) {
          int idx = colPointers[blockPos_c + c] + entriesPerColumn_blockwise[blockIdx_c] + r;
          // fprintf(stderr, "      entry %i %i: %i %i\n", r,c, colPointers[blockPos_c+c], idx);
          rowIndices[idx] = blockPos_r + r;
          data[idx] = it->second[r * BlockSizeRows + c];
          ++numEntries;
        }
      entriesPerColumn_blockwise[blockIdx_c] += BlockSizeRows;
    }
    // fprintf(stderr, "column pointers:\n");
    // for (size_t i = 0; i < entriesPerColumn_blockwise.size()*BlockSizeCols+1; ++i) fprintf(stderr, "%i ",
    // colPointers[i]); fprintf(stderr, "\nrow indices:\n");
    //for (size_t i = 0; i < numEntries; ++i) fprintf(stderr, "%i ", rowIndices[i]);
  }
  /**
   * @brief                将稀疏矩阵填充为密集矩阵
   * @param[out] dest      输出数组（需预先分配空间）
   * @param[in] rowStride  每行的步长（通常等于总列数）
   * @details              直接覆盖所有元素，非零块按位置写入
  */
  void densify(double *dest, int rowStride) const {
    typename MatrixData::const_iterator it = mData.begin();
    for (; it != mData.end(); ++it) {
      int blockPos_r = it->first.block_r * BlockSizeRows;
      int blockPos_c = it->first.block_c * BlockSizeCols;
      //fprintf(stderr, "filling block at %i %i\n", blockPos_r, blockPos_c);
      for (int r = 0; r < BlockSizeRows; ++r)
        for (int c = 0; c < BlockSizeCols; ++c) {
          dest[(blockPos_r + r) * rowStride + blockPos_c + c] = it->second[r * BlockSizeRows + c];
        }
    }
  }

 private:
   MatrixData mData; // 块数据存储容器
};
}