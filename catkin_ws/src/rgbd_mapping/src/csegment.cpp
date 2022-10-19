#include "csegment.h"

CSegment::CSegment()
{
    //clustering planes
    _widthForWallSegment = 0.65f;
    _heightForWallSegment = 0.65f;
    _toleranceForWallDotUp = 0.35f;
    _toleranceForWallFilter = 0.1f;
    //for segmenting ground
    _widthForGroundSegment = 0.5f;
    _heightForGroundSegment = 0.5f;
    _toleranceForGroundDotUp = 0.85f;
    _toleranceForGroundRefineSegment = 0.1f;
    //for Eucleadian segmenting
    _toleranceForClustering = 0.1f;
    _cluster_max_size = 35000;
    _cluster_min_size = 200;
    _objTolerance = 0.05f;
    //for plane segmenting
    _maxIterationForPlaneSegment = 5000;//for plane segment
    _distThreshForPlaneSegment = 0.1f;//for plane segment 0.06
    _rateForPlaneSegment = 0.01f;//for plane segment 0.2
    //for merging walls
    _toleranceForDotBetweenWalls = 0.9f;
    _toleranceForDistBetweenWalls = 0.5f;
    _connect_dist = 0.8f;
    //
    _wallHeight = 2.5f;
    _z_bottom = 0.0f;
    _object_min_thresh = 0.15f;
    _object_max_thresh = 1.6f;
    _object_top_thresh = 0.6f;
    _tolerSizeForCluster = 1000;
    _upper_dist = 0.8f;
    _lower_dist = 0.1f;
    for(int i=0; i < 3; ++i)
    {
       _vOffset1[i] = 0.0f;
       _vOffset2[i] = 0.0f;
    }
    _toleranceFrames = 80;
    _groundPlane[0] = 0.0f;
    _groundPlane[1] = 0.0f;
    _groundPlane[2] = 1.0f;
    _groundPlane[3] = 0.0f;
    _frameCount = 0;
}

void CSegment::setClusteringParams(float objClustTolerance, float cluster_tolerance,
                                   int cluster_max_size, int cluster_min_size, float toleranceForFilter)
{
    _objTolerance = objClustTolerance;
    _toleranceForClustering = cluster_tolerance;
    _cluster_max_size = cluster_max_size;
    _cluster_min_size = cluster_min_size;
    _toleranceForWallFilter = toleranceForFilter;
    _densityForWall = 1/_toleranceForClustering;
    _densityForWall *= _densityForWall;
    _densityForObj = 1/_objTolerance;
    _densityForObj *= _densityForObj*_densityForObj;
    _densityForObj *= 0.05f;
}

void CSegment::setPlaneClusterParams(float planeWidth, float planeHeight, float dotTolerance)
{
    _widthForWallSegment = planeWidth;
    _heightForWallSegment = planeHeight;
    _toleranceForWallDotUp = dotTolerance;
}

void CSegment::setGroundAndWallHeight(float z_bottom, float wallHeight)
{
    _z_bottom = z_bottom;
    _wallHeight = wallHeight;
}

void CSegment::setObjParams(float min_thresh, float max_thresh, float top_thresh, int toleranceFrames)
{
    _object_min_thresh = min_thresh;
    _object_max_thresh = max_thresh;
    _object_top_thresh = top_thresh;
    _toleranceFrames = toleranceFrames;
}

void CSegment::setWalls(std::vector<CQuad> &walls)
{
    _walls = walls;
}

void CSegment::setBoxes(std::vector<CBoundingBox> &boxes)
{
    _boxes = boxes;
}

void CSegment::removingCeilingAndGround(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pCloud1,
                                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pCloud2)
{
    float top = _wallHeight + _z_bottom;
    for(int i=0; i < pCloud1->points.size(); ++i)
    {
        if(pCloud1->points[i].z > _z_bottom && pCloud1->points[i].z < top)
        {
            pCloud2->points.push_back(pCloud1->points[i]);
        }
    }
}

void CSegment::clustering(float clusterTolerance,
                          pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pCloud,
                          std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clusters)
{
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (pCloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance (clusterTolerance);//(_cluster_tolerance); // 2cm
    ec.setMinClusterSize (_cluster_min_size);//10
    ec.setMaxClusterSize (_cluster_max_size);//25000
    ec.setSearchMethod (tree);
    ec.setInputCloud (pCloud);
    ec.extract (cluster_indices);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr maxCluster(new pcl::PointCloud<pcl::PointXYZRGB>);
    int nr = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
       pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
       for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
       {
         cloud_cluster->points.push_back(pCloud->points[*pit]);
       }
       if(cloud_cluster->points.size() > _cluster_min_size)
           clusters.push_back(cloud_cluster);
    }
}

void CSegment::getMaxCluster(float tolerance,
                             pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in,
                             pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out)
{
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (cloud_in);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance (tolerance);//(_cluster_tolerance); // 2cm
    ec.setMinClusterSize (_cluster_min_size);//10
    ec.setMaxClusterSize (_cluster_max_size);//25000
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_in);
    ec.extract (cluster_indices);
    int nr = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
       pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
       for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
       {
         cloud_cluster->points.push_back(cloud_in->points[*pit]);
       }
       if(cloud_cluster->points.size() > nr)
       {
           nr = cloud_cluster->points.size();
           cloud_out = cloud_cluster;
       }
    }
}

int CSegment::main_process(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in,
                            pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out)
{
    static int nFrames = 0;
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    //removingCeilingAndGround(cloud_in, pCloud);
    std::vector<CQuad> walls;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> planarClouds;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr nonPlanarCloud;
    std::vector<Eigen::Vector4f> vCoeffs;
    std::vector<CQuad> candidateWalls;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> candidateWallClouds;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudObjects(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudWalls(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::vector<Eigen::Vector4f> horizenPlanes;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> horizenPlanarClouds;
    extractPlanes(cloud_in, planarClouds, vCoeffs, nonPlanarCloud);
    extractWalls(planarClouds, vCoeffs, walls, cloudWalls, candidateWalls, candidateWallClouds, cloudObjects, horizenPlanes, horizenPlanarClouds);
    extractObjects(horizenPlanes, horizenPlanarClouds, cloudObjects);
    //refineSegment(walls, _walls, candidateWalls, candidateWallClouds, cloudObjects);
    *cloudObjects += *nonPlanarCloud;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> objClusters;
    clustering(_objTolerance,cloudObjects,objClusters);
    std::vector<CBoundingBox> boxes;
    for(int i=0; i < objClusters.size(); ++i)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::copyPointCloud(*objClusters[i], *cloud_xyz);
        CBoundingBox box;
        box.setPointCloud(cloud_xyz);
        box.construct(cloud_xyz,_wall_normals[0], nFrames);
        if(mergeBox(box))
        {
            *cloud_out += *objClusters[i];
            boxes.push_back(box);
        }
    }
    updateBottom(walls, boxes);
    mergeWalls(walls);
    ++nFrames;
    return 1;
}

void CSegment::extractPlanes(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in,
                              std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &planarClouds,
                              std::vector<Eigen::Vector4f> &vCoeffs,
                              pcl::PointCloud<pcl::PointXYZRGB>::Ptr &nonPlanarCloud)
{
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (_maxIterationForPlaneSegment);
    seg.setDistanceThreshold (_distThreshForPlaneSegment);//(0.01);
    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    int nr_points = cloud_in->points.size();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
    while(cloud_in->points.size() > _rateForPlaneSegment*nr_points)
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>);
      // Segment the largest planar component from the remaining cloud
      seg.setInputCloud (cloud_in);
      seg.segment (*inliers, *coefficients);
      if(inliers->indices.size () == 0)
      {
        break;
      }
      Eigen::Vector4f coeff(coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[4]);
      vCoeffs.push_back(coeff);
      extract.setInputCloud (cloud_in);
      extract.setIndices (inliers);
      extract.setNegative (false);
      extract.filter(*cloud_p);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p1 (new pcl::PointCloud<pcl::PointXYZRGB>);
      getMaxCluster(_toleranceForClustering, cloud_p, cloud_p1);
      if(cloud_p1->points.size() > 10)
          planarClouds.push_back(cloud_p1);
      extract.setInputCloud (cloud_in);
      extract.setIndices (inliers);
      extract.setNegative (true);
      extract.filter (*cloud_f);
      cloud_in.swap (cloud_f);
    }
    nonPlanarCloud = cloud_in;
}

int CSegment::extractWalls(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &planarClouds,
                                   std::vector<Eigen::Vector4f> &planes,
                                   std::vector<CQuad> &walls,
                                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloudWalls,
                                   std::vector<CQuad> &candidateWalls,
                                   std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &candidateWallClouds,
                                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr &objClouds,
                                   std::vector<Eigen::Vector4f> &horizenPlanes,
                                   std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &horizenPlanarClouds)
{
    int nState = 1;
    Eigen::Vector3f vUp(0.0f, 0.0f, 1.0f);
    for(int i=0; i < planarClouds.size(); ++i)
    {
       pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>());
       pcl::copyPointCloud(*planarClouds[i], *cloud_xyz);
       Eigen::Vector3f vNormal(planes[i][0], planes[i][1], planes[i][2]);
       vNormal.normalize();
       float dot = fabs(vUp.dot(vNormal));
       if(dot > 0.65f)
       {//detect horizon planes
           horizenPlanes.push_back(planes[i]);
           horizenPlanarClouds.push_back(planarClouds[i]);
           continue;
       }
       if(dot < _toleranceForWallDotUp)
       {
           if(_walls.size()==0)
               setWallNormals(vNormal);
           CQuad quad(_z_bottom, _wallHeight);
           quad.setCoefficients(planes[i][0], planes[i][1], planes[i][2], planes[i][3]);
           if(fabs(vNormal.dot(_wall_normals[0])) > fabs(vNormal.dot(_wall_normals[1])))
           {
               dot = fabs(vNormal.dot(_wall_normals[0]));
               vNormal = _wall_normals[0];
           }
           else
           {
               dot = fabs(vNormal.dot(_wall_normals[1]));
               vNormal = _wall_normals[1];
           }
           quad.setPointCloud(cloud_xyz);
           quad.construct(cloud_xyz, vNormal);
           if(quad.getWidth() > _widthForWallSegment && quad.getHeight() > _heightForWallSegment)
           {
              //if(dot < 0.95f)//pose error
                  //return -1;
              *cloudWalls += *planarClouds[i];
              walls.push_back(quad);
              continue;
           }
           candidateWalls.push_back(quad);
           candidateWallClouds.push_back(planarClouds[i]);
           continue;
       }
       *objClouds += *planarClouds[i];
    }
    return nState;
}

void CSegment::refineSegment(std::vector<CQuad> &walls1,
                             std::vector<CQuad> &walls2,
                             std::vector<CQuad> &candidateWalls,
                             std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &candidateWallClouds,
                             pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloudObj)
{
    for(int i=0; i < candidateWalls.size(); ++i)
    {
        bool flg = false;
        for(int j=0; j < walls1.size(); ++j)
        {
            if(walls1[j].isOverlappedQuads(candidateWalls[i],_toleranceForDotBetweenWalls, _toleranceForWallFilter))
            {
                flg=true;
                break;
            }
        }
        for(int j=0; j < walls2.size(); ++j)
        {
            if(walls2[j].isOverlappedQuads(candidateWalls[i],_toleranceForDotBetweenWalls, _toleranceForWallFilter))
            {
                flg=true;
                break;
            }
        }
        if(flg==false)
        {
            *cloudObj += *candidateWallClouds[i];
        }
    }
}

void CSegment::updateBottom(std::vector<CQuad> &quads, std::vector<CBoundingBox> &boxes)
{
    for(int i=0; i < quads.size(); ++i)
    {
        _z_bottom = MIN(_z_bottom, quads[i].getBottom());
    }
    for(int i=0; i < boxes.size(); ++i)
    {
        _z_bottom = MIN(_z_bottom, boxes[i].getBottom());
    }
}

void CSegment::mergeWalls(std::vector<CQuad> &quads)
{
    for(int i=0; i < quads.size(); ++i)
    {
        _walls.push_back(quads[i]);
    }
    //updateBottom(quads);
    bool bloop = true;
    while(bloop==true)
    {
        bloop = false;
        for(int i=0; i < _walls.size(); ++i)
        {
            for(int j=i+1; j < _walls.size(); ++j)
            {
                Eigen::Vector3f vOffset;
                if(_walls[i].merge(_walls[j], _toleranceForDotBetweenWalls, _toleranceForDistBetweenWalls, vOffset))
                {
                    _walls.erase(_walls.begin()+j);
                    bloop = true;
                    break;
                }
            }
            if(bloop==true)
                break;
        }
    }
    for(int i=0; i < _walls.size(); ++i)
    {
        _walls[i].update(_z_bottom, _wallHeight);
    }
    correctingQuads();
}

void CSegment::updateGround(std::vector<Eigen::Vector3f> &vertices)
{
    float z_bottom = 0.0f;
    for(int i=0; i < vertices.size(); ++i)
    {
        z_bottom += vertices[i][2];
    }
    z_bottom /= vertices.size();
    if(z_bottom > _z_bottom)
        _z_bottom = z_bottom;
}

bool CSegment::isObjectBox(CBoundingBox &box)
{
    if(box.getMinDimension() < _object_min_thresh)
        return false;
    if(box.getTop(_z_bottom) > _object_top_thresh)
        return false;
    if(box.getHeight() > 0.8f)
        return false;
    float max_dim = box.getMaxDimension();
    if(max_dim > _object_max_thresh)
        return false;
    if(box.getDensity() < _densityForObj)
        return false;
    return true;
}

bool CSegment::mergeBox(CBoundingBox &box)
{
    if(isObjectBox(box)==false)
       return false;
    _boxes.push_back(box);
    bool bloop = true;
    while(bloop==true)
    {
        bloop = false;
        for(int i=0; i < _boxes.size(); ++i)
        {
            for(int j=i+1; j < _boxes.size(); ++j)
            {
                if(_boxes[i].merge(_boxes[j], _wall_normals[0], _toleranceFrames))
                {
                    _boxes.erase(_boxes.begin()+j);
                    bloop = true;
                    break;
                }
            }
            if(bloop==true)
                break;
        }
    }
    bloop = true;
    while(bloop==true)
    {
        bloop = false;
        for(int i=0; i < _boxes.size(); ++i)
        {
            if(_boxes[i].getTop(_z_bottom) > _object_top_thresh)
            {
                _boxes.erase(_boxes.begin()+i);
                bloop = true;
                break;
            }
        }
    }
    return true;
}

int CSegment::checkPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCloud,
                         float c0, float c1, float c2, float c3)
{
    return 0;
}

bool CSegment::isWall(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCloud,
                      float c0, float c1, float c2, float c3)
{
    Eigen::Vector3f vUp(0.0f, 0.0f, 1.0f);
    Eigen::Vector3f vNormal(c0, c1, c2);
    vNormal.normalize();
    if(fabs(vNormal.dot(vUp)) > _toleranceForWallDotUp)
        return false;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud(*pCloud, *cloud_xyz);
    CQuad wall(_z_bottom, _wallHeight);
    wall.setCoefficients(c0, c1, c2, c3);
    wall.construct(cloud_xyz,vNormal);
    float width = wall.getWidth();
    float height = wall.getHeight();
    if(width > _widthForWallSegment && height > _heightForWallSegment)
    {
        if(_walls.size()==0)
        {
            Eigen::Vector3f vNormal(c0, c1, c2);
            vNormal.normalize();
            getWallNormals(vNormal);
        }
        _walls.push_back(wall);
        return true;
    }
    return false;
}

bool CSegment::isGround(std::vector<Eigen::Vector3f> &vertices)
{
    float z = 0;
    for(int i=0; i < vertices.size(); ++i)
    {
        z += vertices[i][2];
    }
    z /= vertices.size();
    if(z <= _z_bottom + 0.01f)
        return true;
    return false;
}

void CSegment::getWallNormals(Eigen::Vector3f &vDir)
{
    Eigen::Vector3f vUp(0.0f, 0.0f, 1.0f);
    Eigen::Vector3f vNormal = vUp.cross(vDir);
    _wall_normals[0] = vNormal.cross(vUp);
    _wall_normals[0].normalize();
    if(_wall_normals[0].dot(vNormal)<0.0f)
        _wall_normals[0] = -_wall_normals[0];
    _wall_normals[1] = vUp.cross(_wall_normals[0]);
    _wall_normals[1].normalize();
}

void CSegment::setWallNormals(Eigen::Vector3f &vDir)
{
    Eigen::Vector3f vUp(0.0f, 0.0f, 1.0f);
    Eigen::Vector3f vNormal = vUp.cross(vDir);
    _wall_normals[0] = vNormal.cross(vUp);
    _wall_normals[0].normalize();
    if(_wall_normals[0].dot(vNormal)<0.0f)
        _wall_normals[0] = -_wall_normals[0];
    _wall_normals[1] = vUp.cross(_wall_normals[0]);
    _wall_normals[1].normalize();
}

void CSegment::getWallsAndBoxes(std::vector<CQuad> &walls, std::vector<CBoundingBox> &boxes)
{
    walls = _walls;
    boxes = _boxes;
}

void CSegment::correctingQuads()
{
    for(int i=0; i < _walls.size(); ++i)
    {
        for(int j=0; j < _walls.size(); ++j)
        {
            if(i==j)
                continue;
            Eigen::Vector3f vNormal2;
            _walls[j].getNormal(vNormal2);
            std::vector<Eigen::Vector3f> vertices2;
            _walls[j].getVertices(vertices2);
            _walls[i].correct(_walls[j], _connect_dist);
        }
    }
}

void CSegment::getOffset(Eigen::Vector3f &vOffset1, Eigen::Vector3f &vOffset2)
{
    vOffset1 = _vOffset1;
    vOffset2 = _vOffset2;
}

bool CSegment::checkIntersectWallAndBox(CBoundingBox &box, std::vector<CQuad> &walls)
{
    for(int j=0; j < walls.size(); ++j)
    {
       if(box.checkIntersectWall(walls[j]))
          return true;
    }
    return false;
}

void CSegment::extractObjects(std::vector<Eigen::Vector4f> &horizenPlanes,
                             std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &planeClouds,
                             pcl::PointCloud<pcl::PointXYZRGB>::Ptr objCloud)
{
    if(horizenPlanes.size() < 2)
        return;
    float bottom = FLT_MAX;
    int c=0;
    for(int i=0; i < horizenPlanes.size(); ++i)
    {
        float z = getBottom(planeClouds[i]);
        if(z < bottom)
        {
            c = i;
            bottom = z;
        }
    }
    for(int i=0; i < horizenPlanes.size(); ++i)
    {
        if(i!=c)
            *objCloud += *planeClouds[i];
    }
}
