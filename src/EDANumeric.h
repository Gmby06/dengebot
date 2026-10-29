#ifndef EDA_NUMERIC_H
#define EDA_NUMERIC_H
#include <math.h>
// C++11 / AVR compatible finite check, without platform-specific isfinite macros.
namespace eda_detail {
inline bool finiteFloat(float x) { return x == x && x != INFINITY && x != -INFINITY; }
}
#endif
