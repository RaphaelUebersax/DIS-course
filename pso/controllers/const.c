#include "const.h"

#if WORLD == 1
const position_t MIGR[] = {{2., 0.5, 0}};
const position_t INIT_POS[] = {
    {-2.9, 0., -M_PI / 2},
    {-2.9, -0.1, -M_PI / 2},
    {-2.9, 0.1, -M_PI / 2},
    {-2.9, -0.2, -M_PI / 2},
    {-2.9, 0.2, -M_PI / 2},
};
#elif WORLD == 2
const position_t MIGR[] = {{-1.9, 0, 0}, {-0.1, 0, 0}};
const position_t INIT_POS[] = {
    {-0.1, 0, M_PI / 2},
    {-0.1, 0.1, M_PI / 2},
    {-0.1, -0.1, M_PI / 2},
    {-0.1, 0.2, M_PI / 2},
    {-0.1, -0.2, M_PI / 2},
    {-1.9, 0, -M_PI / 2},
    {-1.9, 0.1, -M_PI / 2},
    {-1.9, -0.1, -M_PI / 2},
    {-1.9, 0.2, -M_PI / 2},
    {-1.9, -0.2, -M_PI / 2},
};
#endif