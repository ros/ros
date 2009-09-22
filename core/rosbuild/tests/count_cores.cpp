#include <stdio.h>
#include <boost/thread/thread.hpp>

int
main(void)
{
  printf("%d\n", boost::thread::hardware_concurrency());
  return 0;
}
