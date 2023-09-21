#include <iostream>
#include <vector>
using namespace std;

int main() {
  int localSize = 256;
  vector<int> nums(localSize, 1);
  int s1, s2;
  for (s1 = 1, s2 = 1; s1 < localSize; s1 <<= 1) {
    for (int localId = 0; localId < localSize; localId++) {
      s2 |= s1;
      if ((localId & s2) == s2){
        nums[localId] += nums[localId - s1];
        printf("localId %d\ts1 s2 %d %d\n", localId, s1, s2);
      }
    }
    printf("=====以上for循环并行发生=====\n");
  }

  for(auto &n:nums)
    cout << n << " ";
  cout << endl;
  return 0;
}
