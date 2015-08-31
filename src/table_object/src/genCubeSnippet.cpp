pcl::PointCloud<pcl::PointXYZ>::Ptr genTestCube(double block_w, double block_d, double block_h){
    const int nLongSideHighRes = 30; // high res
    const int nLongSideLowRes = 27; // low res

    const int nShortSideHighRes = 35; // high res
    const int nShortSideLowRes = 9; // low res
        
    pcl::PointCloud<pcl::PointXYZ>::Ptr cubeCloud (new pcl::PointCloud<pcl::PointXYZ>);


    int npointsBottomFace = nShortSideHighRes* nShortSideHighRes;
    int npointsSideFace = nShortSideHighRes* nLongSideHighRes;
    int npointsTotal = 2*npointsBottomFace + 4 *npointsSideFace;
    
    float dxShortSide1 = block_w / (float)nShortSideHighRes;
    float dxShortSide2 = block_d / (float)nShortSideHighRes;
    float dxLongSide =  block_h /  (float)nLongSideHighRes;
    
    std::cerr << "# dxShortSide1: " << dxShortSide1 << std::endl;
    std::cerr << "# dxShortSide2: " << dxShortSide2 << std::endl;
    std::cerr << "# dxLongSide: " << dxLongSide << std::endl;
    std::cerr << "# npointsTotal: " << npointsTotal << std::endl;
    
    cubeCloud->width = npointsTotal;
    cubeCloud->height = 1;
    cubeCloud->points.resize(npointsTotal); // allocate space for all the points we need
    
    // make the top and bottom cap faces
    // these go at z = +- block_h /2 
    // from x,y = -w/2, -d/2  to x,z = w/2, d/2
    int counter = 0;
    float xval, yval, zval;
    float xOffset, yOffset, zOffset;
    
    // top face
    zval = block_h / 2;
    xOffset = - block_d / 2;
    yOffset = - block_w / 2;
    for(int i = 0; i < nShortSideHighRes; i++){
        for(int j = 0; j < nShortSideHighRes; j++){
            cubeCloud->points[counter].x = i*dxShortSide2 + xOffset;
            cubeCloud->points[counter].y = j*dxShortSide1 + yOffset;
            cubeCloud->points[counter].z = zval;
            counter++;
        }
    }

    // bottom face
    zval = -block_h / 2;
    xOffset = - block_d / 2;
    yOffset = - block_w / 2;
    for(int i = 0; i < nShortSideHighRes; i++){
        for(int j = 0; j < nShortSideHighRes; j++){
            cubeCloud->points[counter].x = i*dxShortSide2 + xOffset;
            cubeCloud->points[counter].y = j*dxShortSide1 + yOffset;
            cubeCloud->points[counter].z = zval;
            counter++;
        }
    }


    // make each side plane
    // 1) y= -block_w/2, x: from - block_d/2 to block_d/2, z from: -block_h/2, to block_h/2
    // 2) y= block_w/2, x: from - block_d/2 to block_d/2, z from: -block_h/2, to block_h/2
    // 3) x = -block_d/2, y: from - block_w/2 to block_w/2, z from: -block_h/2, to block_h/2
    // 4) x = block_d/2, y: from - block_w/2 to block_w/2, z from: -block_h/2, to block_h/2

    yval = -block_w / 2;
    xOffset = - block_d / 2;
    zOffset = - block_h / 2;
    for(int i = 0; i < nShortSideHighRes; i++){
        for(int j = 0; j < nLongSideHighRes; j++){
            cubeCloud->points[counter].x = i*dxShortSide2 + xOffset;
            cubeCloud->points[counter].y = yval;
            cubeCloud->points[counter].z = j*dxLongSide + zOffset;;
            counter++;
        }
    }

    yval = block_w / 2;
    xOffset = - block_d / 2;
    zOffset = - block_h / 2;
    for(int i = 0; i < nShortSideHighRes; i++){
        for(int j = 0; j < nLongSideHighRes; j++){
            cubeCloud->points[counter].x = i*dxShortSide2 + xOffset;
            cubeCloud->points[counter].y = yval;
            cubeCloud->points[counter].z = j*dxLongSide + zOffset;;
            counter++;
        }
    }

    xval = -block_d / 2;
    yOffset = - block_w / 2;
    zOffset = - block_h / 2;
    for(int i = 0; i < nShortSideHighRes; i++){
        for(int j = 0; j < nLongSideHighRes; j++){
            cubeCloud->points[counter].x = xval;
            cubeCloud->points[counter].y = i*dxShortSide1 + yOffset;
            cubeCloud->points[counter].z = j*dxLongSide + zOffset;
            counter++;
        }
    }

    xval = block_d / 2;
    yOffset = - block_w / 2;
    zOffset = - block_h / 2;
    for(int i = 0; i < nShortSideHighRes; i++){
        for(int j = 0; j < nLongSideHighRes; j++){
            cubeCloud->points[counter].x = xval;
            cubeCloud->points[counter].y = i*dxShortSide1 + yOffset;
            cubeCloud->points[counter].z = j*dxLongSide + zOffset;
            counter++;
        }
    }
    

    return cubeCloud;
}