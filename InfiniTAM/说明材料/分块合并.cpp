#include <iostream>
#include <vector>
using namespace std;

int main() {
  int localSize = 256;
  vector<int> prefixBuffer(localSize, 1);
  int s1, s2;
  //! 步骤一
  for (s1 = 1, s2 = 1; s1 < localSize; s1 <<= 1) { // 实现一
    for (int localId = 0; localId < localSize; localId++) {        
      s2 |= s1;
      if ((localId & s2) == s2){
        prefixBuffer[localId] += prefixBuffer[localId - s1];
        printf("localId %d\ts1 s2 %d %d\n", localId, s1, s2);
      }
    }
    printf("=====以上for循环并行发生=====\n");
  }

  // for (s1 = 1; s1 < localSize; s1 <<= 1) {       // 实现二
  //   for (int localId = 0; localId < localSize; localId++) {
  //     if ((localId + 1) % (s1*2) == 0) { // +1是因为localId从0开始
  //       prefixBuffer[localId] += prefixBuffer[localId - s1];
  //       printf("localId %d\ts1 %d\n", localId, s1);
  //     }
  //   }
  //   printf("=====以上for循环并行发生=====\n");
  // }

  for(auto &n:prefixBuffer)
    cout << n << " ";
  cout << endl;
  printf("###############################################################\n");
  //! 步骤二
  for (s1 >>= 2, s2 >>= 1; s1 >= 1; s1 >>= 1, s2 >>= 1) { // 实现一
    for (int localId = 0; localId < localSize; localId++) {
      if (localId != localSize - 1 && (localId & s2) == s2) {
        prefixBuffer[localId + s1] += prefixBuffer[localId];
        printf("localId %d\ts1 s2 %d %d\n", localId, s1, s2);
      }
    }
    printf("=====以上for循环并行发生=====\n");
  }

  // for (s1 = localSize / 2; s1 >= 2; s1 /= 2) {            // 实现二
  //   for (int localId = 0; localId < localSize; localId++) {
  //     if (localId != localSize - 1 && (localId + 1) % s1 == 0) { // +1是因为localId从0开始
  //       prefixBuffer[localId + s1/2] += prefixBuffer[localId];
  //       printf("localId %d\ts1 %d\n", localId, s1);
  //     }
  //   }
  //   printf("=====以上for循环并行发生=====\n");
  // }

  for(auto &n:prefixBuffer)
    cout << n << " ";
  cout << endl;
  return 0;
}
