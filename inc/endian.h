#ifndef ENDIAN_H
#define ENDIAN_H

#ifndef htobe32
#if defined(__BYTE_ORDER__) && (__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__)
#define htobe32(x)      __builtin_bswap32(x)
#define htobe16(x)      __builtin_bswap16(x)
#define htole32(x)      (x)
#define htole16(x)      (x)
#else
#define htobe32(x)      (x)
#define htobe16(x)      (x)
#define htole32(x)      __builtin_bswap32(x)
#define htole16(x)      __builtin_bswap16(x)
#endif
#endif

#ifndef betoh32
#if defined(__BYTE_ORDER__) && (__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__)
#define betoh32(x)      __builtin_bswap32(x)
#define betoh16(x)      __builtin_bswap16(x)
#define letoh32(x)      (x)
#define letoh16(x)      (x)
#else
#define betoh32(x)      (x)
#define betoh16(x)      (x)
#define letoh32(x)      __builtin_bswap32(x)
#define letoh16(x)      __builtin_bswap16(x)
#endif
#endif

#endif