
#include <time.h>

struct cptr_time_t {
  struct timespec resolution ;
  struct timespec t          ;
  char*  type                ;
}

