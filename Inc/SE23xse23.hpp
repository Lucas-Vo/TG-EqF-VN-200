#include <groups/SEn3.hpp>
#include "linalgtypes.hpp"

// lie
using SE23 = group::SEn3<double, 2>;
using se23 = SE23::MatrixType; // 5x5 algebra matrix (wedge)
using SO3 = group::SO3<double>;

typedef struct
{
    SE23 pose;
    se23 bias;
} SE23xse23; // SE2 (3) × se2(3)

typedef struct
{
    se23 pose;
    se23 bias;
} se23xse23; // se2 (3) × se2(3)

// group helpers
SE23xse23 ExpSE23xse23(const se23xse23 &xi);
SE23xse23 MulSE23xse23(const SE23xse23 &X, const SE23xse23 &Y);