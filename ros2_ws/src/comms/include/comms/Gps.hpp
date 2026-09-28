#include <cstdint>

// Match the byte-for-byte layout of the serialized GPS payloads.
#pragma pack(push, 1)
typedef struct posPayload
{
  uint32_t iTow;
  int32_t longitude;
  int32_t latitude;
  int32_t height;
  uint32_t hMsl;
  uint32_t hAcc;
  uint32_t vAcc;
} posPayload;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct covPayload
{
  uint32_t iTow;
  uint8_t version;
  uint8_t posCovValid;
  uint8_t velCovValid;
  uint64_t paddingA;
  uint8_t paddingB;
  float posCovNN;
  float posCovNE;
  float posCovND;
  float posCovEE;
  float posCovED;
  float posCovDD;
  float velCovNN;
  float velCovNE;
  float velCovND;
  float velCovEE;
  float velCovED;
  float velCovDD;
} covPayload;
#pragma pack(pop)
