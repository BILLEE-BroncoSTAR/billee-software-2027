#include <cstdint>

// avoigd struct packing so we can easily copy the buffers into the struct
#pragma pack(push, 1)
typedef struct postPayload{
  uint32_t iTow;
  int32_t longitude;
  int32_t latitude;
  int32_t height;
  uint32_t hMsl;
  uint32_t vAcc;
} posPayload;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct covPayload{
  uint32_t iTow;
  uint8_t version;
  uint8_t posCovValid;
  uint8_t velCovValid;
  uint64_t paddingA;
  uint8_t paddingB;
  float posCovNH;
  float posCovNE;
  float posCovND;
  float posCovEE;
  float posCovED;
  float posCovDD;
  float velCovNH;
  float velCovNE;
  float velCovND;
  float velCovEE;
  float velCovED;
  float velCovDD;
} covPayload;

#pragma pack(pop)