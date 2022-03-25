#include "GroundRemoval.h"

int main(void)
{
    GroundRemoval ground_removal;

    ground_removal.readPCDFile("/home/az/下载/task6样例/label/1641974142700.pcd");
    ground_removal.process();
    ground_removal.visualizationGroundRemoval();
    ground_removal.saveToPCD("ground.pcd","no_ground.pcd");


    return 0;

}