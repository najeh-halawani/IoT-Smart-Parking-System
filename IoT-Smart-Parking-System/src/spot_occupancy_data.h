#ifndef SPOT_OCCUPANCY_DATA_H
#define SPOT_OCCUPANCY_DATA_H

struct SpotOccupancyData {
    int spotId;
    bool occupied;
    float usDistance;
    float tofDistance;
};

#endif // SPOT_OCCUPANCY_DATA_H
