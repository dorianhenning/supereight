//
// Created by dorianhenning on 09/11/18.
//

#include "se/ObjectMappingSystem.h"
//#include <se/DenseSLAMSystem.h>
#include <se/ray_iterator.hpp>
#include <se/algorithms/meshing.hpp>
#include <se/geometry/octree_collision.hpp>
#include <se/vtk-io.h>
#include "timings.h"
#include <perfstats.h>
#include "preprocessing.cpp"
#include "tracking.cpp"
#include "rendering.cpp"
#include "bfusion/mapping_impl.hpp"
#include "kfusion/mapping_impl.hpp"
#include "bfusion/alloc_impl.hpp"
#include "kfusion/alloc_impl.hpp"
#include "object/Object.cpp"
#include "segmentation/Segmentation.cpp"
