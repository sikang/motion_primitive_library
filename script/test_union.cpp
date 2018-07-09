#include <iostream>
#include <bitset>

namespace Control {
enum Control {
  SNP = 0b1111,
  VEL = 0b1000,
  ACC = 0b1100,
  JRK = 0b1110,
};
}

typedef union {
  struct {
    bool use_jrk: 1;
    bool use_acc: 1;
    bool use_vel: 1;
    bool use_pos: 1;
  };

  Control::Control use_xxx : 4;

} USE;

void print(USE u) {
  std::cout << "use_pos | use_vel | use_acc | use_jrk: " << std::endl;
  std::cout << u.use_pos << " | " << u.use_vel << " | " << u.use_acc << " | " << u.use_jrk << std::endl;
  std::bitset<4> x(u.use_xxx);
  std::cout << "raw use_xxx: " << u.use_xxx << std::endl;
  std::cout << "use_xxx: " << x << std::endl;
  if(u.use_xxx == Control::VEL)
    std::cout << "use vel!" << std::endl;
  else if(u.use_xxx == Control::ACC)
    std::cout << "use acc!" << std::endl;
  else if(u.use_xxx == Control::JRK)
    std::cout << "use jrk!" << std::endl;
  else if(u.use_xxx == Control::SNP)
    std::cout << "use snp!" << std::endl;
  else
    std::cout << "use null!" << std::endl;
  //std::cout << "size of Control: " << sizeof(u.use_xxx) << std::endl;
}

int main() {


  USE u1, u2, u3, u4, u5;

  print(u1);

  u2.use_pos = true;
  u2.use_vel = false;
  u2.use_acc = false;
  u2.use_jrk = false;

  print(u2);

  u3.use_pos = true;
  u3.use_vel = true;
  u3.use_acc = false;
  u3.use_jrk = false;

  print(u3);

  u4.use_pos = true;
  u4.use_vel = true;
  u4.use_acc = true;
  u4.use_jrk = false;

  print(u4);

  u5.use_pos = true;
  u5.use_vel = true;
  u5.use_acc = true;
  u5.use_jrk = true;

  print(u5);

  std::cout << "VEL_CONTROL: " << Control::VEL << std::endl;
  std::cout << "ACC_CONTROL: " << Control::ACC << std::endl;
  std::cout << "JRK_CONTROL: " << Control::JRK << std::endl;
  std::cout << "SNP_CONTROL: " << Control::SNP << std::endl;
  return 0;
}
