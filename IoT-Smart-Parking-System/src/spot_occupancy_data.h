#pragma once

struct SpotOccupancyData {
    int spotId;
    bool occupied;
    float usDistance;
    float tofDistance;
    time_t timestamp;
};