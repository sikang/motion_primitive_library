/**
 * @file control.h
 * @brief Control classes
 */
#ifndef MPL_CONTROL_H
#define MPL_CONTROL_H

namespace Control {
  /// Enum for control input
  enum Control {
    VEL = 0b10000,///<control input is vel
    ACC = 0b11000,///<control input is acc
    JRK = 0b11100,///<control input is jrk
    SNP = 0b11110,///<control input is snp
    VELxYAW = 0b10001,///<control input is vel and yaw
    ACCxYAW = 0b11001,///<control input is acc and yaw
    JRKxYAW = 0b11101,///<control input is jrk and yaw
    SNPxYAW = 0b11111 ///<control input is snp and yaw
  };
}
#endif
