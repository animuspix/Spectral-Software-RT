
#include "sampler.h"

sampler::philox sampler::rand_streams[parallel::numTiles];