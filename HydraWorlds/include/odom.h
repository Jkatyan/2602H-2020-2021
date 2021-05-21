#ifndef ODOM_H
#define ODOM_H

extern float x,y,angle,diff;

extern bool trackenabled;

void track();
void stopTracking();
#endif
